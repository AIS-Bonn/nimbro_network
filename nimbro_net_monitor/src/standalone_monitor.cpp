// Standalone network monitor for non-ROS devices (e.g. routers)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <fstream>
#include <stdexcept>
#include <optional>
#include <fstream>
#include <filesystem>
#include <chrono>

#include <ifaddrs.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>

#include <fmt/format.h>

#include "../contrib/archive/archive.h"

#include "route.h"
#include "nl80211.h"
#include "rtnetlink.h"

#include "standalone_proto.h"


struct Interface
{
	Interface(const std::string& name, NL80211& nl)
	 : name(name)
	 , m_nl{nl}
	{
		// Read speed
		{
			if(auto speed = readProp("speed"))
				bitsPerSecond = (*speed) * 1000ULL * 1000ULL;
		}

		// Is it duplex?
		{
			char fname[256];
			snprintf(fname, sizeof(fname), "/sys/class/net/%s/duplex", name.c_str());
			std::ifstream stream(fname);
			if(stream)
			{
				std::string str;
				stream >> str;
				if(str == "full")
					duplex = true;
			}
		}

		m_isWireless = std::filesystem::exists(fmt::format("/sys/class/net/{}/wireless", name));
		if(m_isWireless)
		{
			if(auto idx = m_nl.getInterfaceByName(name))
				m_wirelessIdx = *idx;
			else
				m_isWireless = false;
		}
	}

// GCC produces a spurious warning here, see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=80635
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
	std::optional<uint64_t> readProp(const std::string& prop) const
	{
		char fname[256];
		snprintf(fname, sizeof(fname), "/sys/class/net/%s/%s", name.c_str(), prop.c_str());

		std::ifstream stream(fname);
		if(!stream)
			return {};

		uint64_t val;
		stream >> val;

		if(stream.bad() || stream.fail())
			return {};

		return val;
	}
#pragma GCC diagnostic pop

	void updateStats(const RTNetlink::Stats& linkStats)
	{
		auto now = std::chrono::steady_clock::now();

		double timeDelta = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastStatsTime).count();
		rx_bandwidth = 8 * (linkStats.rx_bytes - lastStats.rx_bytes) / timeDelta;
		tx_bandwidth = 8 * (linkStats.tx_bytes - lastStats.tx_bytes) / timeDelta;
		rx_packet_rate = (linkStats.rx_packets - lastStats.rx_packets) / timeDelta;
		tx_packet_rate = (linkStats.tx_packets - lastStats.tx_packets) / timeDelta;

		if(m_isWireless)
		{
			while(true)
			{
				try
				{
					if(auto stats = m_nl.getStats(m_wirelessIdx))
						wifiStats = std::move(*stats);
				}
				catch(NL80211::RetryException&)
				{
					fmt::print(stderr, "Retrying interrupted NL80211 dump\n");
					continue;
				}

				break;
			}
		}

		lastStats = linkStats;
		lastStatsTime = now;
	}

	std::string name;
	uint64_t bitsPerSecond = 0;
	bool duplex = false;

	RTNetlink::Stats lastStats;

	double rx_bandwidth = 0;
	double rx_packet_rate = 0;
	double tx_bandwidth = 0;
	double tx_packet_rate = 0;

	proto::WifiStats wifiStats;

	std::chrono::steady_clock::time_point lastStatsTime;

private:
	NL80211& m_nl;
	bool m_isWireless = false;
	unsigned int m_wirelessIdx = 0;
};

class NetMonitor
{
public:
	explicit NetMonitor(const std::string& destination_host)
	{
		addrinfo hints{};
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_DGRAM;
		hints.ai_protocol = 0;


		addrinfo* res = nullptr;
		if(getaddrinfo(destination_host.c_str(), "5102", &hints, &res) != 0)
		{
			fmt::print(stderr, "ERROR: Could not lookup destination host '{}': {}\n", destination_host, strerror(errno));
			std::exit(1);
		}
		if(!res)
		{
			fmt::print(stderr, "ERROR: Could not lookup destination host '{}'\n", destination_host);
			std::exit(1);
		}

		m_socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
		if(m_socket < 0)
		{
			fmt::print(stderr, "ERROR: Could not create socket: {}\n", strerror(errno));
			freeaddrinfo(res);
			std::exit(1);
		}

		if(connect(m_socket, res->ai_addr, res->ai_addrlen) != 0)
		{
			fmt::print(stderr, "ERROR: Could not connect() socket to destination '{}': {}\n", destination_host, strerror(errno));
			freeaddrinfo(res);
			std::exit(1);
		}

		freeaddrinfo(res);

		ifaddrs* addrs = nullptr;
		if(getifaddrs(&addrs) != 0)
		{
			fmt::print(stderr, "ERROR: Could not get local interfaces: {}\n", strerror(errno));
			std::exit(1);
		}

		for(ifaddrs* addr = addrs; addr; addr = addr->ifa_next)
		{
			m_interfaces.insert(std::make_pair(std::string{addr->ifa_name}, Interface(addr->ifa_name, m_nl80211)));
		}

		freeifaddrs(addrs);

		updateStats();
	}

	void updateStats()
	{
		auto linkStats = m_rtnl.getLinkStats();
		RTNetlink::Stats dummyStats;

		for(auto& iface : m_interfaces)
		{
			auto it = linkStats.find(iface.first);
			if(it != linkStats.end())
				iface.second.updateStats(it->second);
			else
				iface.second.updateStats(dummyStats);
		}

		// Generate message
		proto::NetworkStats stats;

		auto now = std::chrono::system_clock::now();
		auto duration = now.time_since_epoch();
		stats.stamp_secs = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
		stats.stamp_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000ULL;

		{
			char buf[256];
			gethostname(buf, sizeof(buf));
			buf[sizeof(buf)-1] = 0;
			stats.host = buf;
		}

		for(auto& ifacePair : m_interfaces)
		{
			auto& iface = ifacePair.second;

			proto::Interface ifaceStats;

			ifaceStats.interface_name = iface.name;
			ifaceStats.max_bits_per_second = iface.bitsPerSecond;
			ifaceStats.duplex = iface.duplex;
			ifaceStats.rx_bandwidth = iface.rx_bandwidth;
			ifaceStats.tx_bandwidth = iface.tx_bandwidth;
			ifaceStats.rx_packet_rate = iface.rx_packet_rate;
			ifaceStats.tx_packet_rate = iface.tx_packet_rate;

			ifaceStats.wifi = ifacePair.second.wifiStats;

			stats.interfaces.push_back(std::move(ifaceStats));
		}

		std::stringstream ss;
		Archive<std::stringstream> a(ss);

		a << stats;

		std::string buf = ss.str();

		int ret = send(m_socket, buf.c_str(), buf.size(), 0);
		if(ret != static_cast<int>(buf.size()))
		{
			fmt::print(stderr, "ERROR: Could not send packet: {}\n", strerror(errno));
		}
	}
private:
	std::map<std::string, Interface> m_interfaces;

	route::Cache m_routeCache;

	RTNetlink m_rtnl;
	NL80211 m_nl80211;

	int m_socket = -1;
};

int main(int argc, char** argv)
{
	if(argc < 2 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
	{
		fmt::print(stderr, "Usage: standalone_monitor [-d] <destination address>\n");
		fmt::print(stderr, "\n");
		fmt::print(stderr, "  -d: ignore SIGHUP\n");
		return 1;
	}

	char* host = argv[1];
	if(argc == 3 && strcmp(argv[1], "-d") == 0)
	{
		struct sigaction action{};
		action.sa_handler = SIG_IGN;
		sigaction(SIGHUP, &action, nullptr);

		host = argv[2];
	}

	NetMonitor monitor{host};

	fmt::print("Sending packets to {}\n", host);

	while(1)
	{
		monitor.updateStats();
		usleep(100 * 1000);
	}

	return 0;
}
