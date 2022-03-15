// Receive & republish data from the standalone_monitor to ROS
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>
#include <rosfmt/full.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <nimbro_net_monitor/NetworkStats.h>

#include "standalone_proto.h"
#include "../contrib/archive/archive.h"

static char buf[4096];

int main(int argc, char** argv)
{
	ros::init(argc, argv, "standalone_receiver", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<nimbro_net_monitor::NetworkStats>("/network/monitor", 3);

	int sock = socket(AF_INET6, SOCK_DGRAM, 0);
	if(sock < 0)
	{
		ROSFMT_ERROR("Could not create socket: {}\n", strerror(errno));
		return 1;
	}

	sockaddr_in6 addr{};
	addr.sin6_family = AF_INET6;
	addr.sin6_addr = in6addr_any;
	addr.sin6_port = htons(5102);

	if(bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
	{
		ROSFMT_ERROR("Could not bind socket: {}\n", strerror(errno));
		return 1;
	}

	while(1)
	{
		int bytes = recv(sock, buf, sizeof(buf), 0);
		if(bytes < 0)
		{
			ROSFMT_ERROR("Could not receive msg: {}\n", strerror(errno));
			return 1;
		}

		std::stringstream ss(std::string(buf, bytes));
		Archive<std::stringstream> a(ss);

		proto::NetworkStats stats;
		a >> stats;

		nimbro_net_monitor::NetworkStats msg;
		msg.header.stamp = ros::Time(stats.stamp_secs, stats.stamp_nsecs);
		msg.host = stats.host;

		for(auto& iface : stats.interfaces)
		{
			auto& msg_iface = msg.interfaces.emplace_back();

			msg_iface.interface_name = iface.interface_name;
			msg_iface.max_bits_per_second = iface.max_bits_per_second;
			msg_iface.duplex = iface.duplex;
			msg_iface.tx_bandwidth = iface.tx_bandwidth;
			msg_iface.rx_bandwidth = iface.rx_bandwidth;
			msg_iface.wifi.is_wifi_device = iface.wifi.is_wifi_device;
			msg_iface.wifi.associated = iface.wifi.associated;
			msg_iface.wifi.associated_since = ros::Duration{}.fromSec(iface.wifi.associated_since);
			msg_iface.wifi.ssid = iface.wifi.ssid;
			msg_iface.wifi.frequency_mhz = iface.wifi.frequency_mhz;
			msg_iface.wifi.signal_dbm = iface.wifi.signal_dbm;
			msg_iface.wifi.signal_avg_dbm = iface.wifi.signal_avg_dbm;
			msg_iface.wifi.beacon_signal_dbm = iface.wifi.beacon_signal_dbm;
			msg_iface.wifi.tx_mcs = iface.wifi.tx_mcs;
			msg_iface.wifi.tx_bw = iface.wifi.tx_bw;
			msg_iface.wifi.tx_bitrate = iface.wifi.tx_bitrate;
			msg_iface.wifi.rx_mcs = iface.wifi.rx_mcs;
			msg_iface.wifi.rx_bw = iface.wifi.rx_bw;
			msg_iface.wifi.rx_bitrate = iface.wifi.rx_bitrate;
		}

		pub.publish(msg);
	}

	return 0;
}
