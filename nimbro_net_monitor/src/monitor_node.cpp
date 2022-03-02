// ROS component
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>
#include <ros/master.h>
#include <ros/network.h>

#include <fstream>
#include <stdexcept>
#include <optional>

#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpc.h>

#include <nimbro_net_monitor/NetworkStats.h>

#include "route.h"

using XMLRPCManager = ros::XMLRPCManager;

static bool slaveCall(const std::string& host, uint32_t port, const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response)
{
	XmlRpc::XmlRpcClient *c = XMLRPCManager::instance()->getXMLRPCClient(host, port, "/");

	if (!c->execute(method.c_str(), request, response))
	{
		XMLRPCManager::instance()->releaseXMLRPCClient(c);
		return false;
	}

	XMLRPCManager::instance()->releaseXMLRPCClient(c);
	return true;
}

struct ConnectionInfo
{
	enum class Direction
	{
		In, Out, Both
	};

	int id = -1;
	std::string destination;
	Direction direction = Direction::Both;
	std::string transport;
	std::string topic;

	bool bytesValid = false;
	int32_t bytes = 0;
	uint64_t bitsPerSecond = 0;

	std::string localNode;
	bool active = false;

	std::string hash() const
	{
		std::stringstream ss;
		ss << topic << "%" << localNode << "%" << id;
		return ss.str();
	}

	static ConnectionInfo fromXmlRpc(XmlRpc::XmlRpcValue& in)
	{
		// Format: {2,/rosout,o,TCPROS,/rosout,1,TCPROS connection on port 46287 to [127.0.0.1:42256 on socket 24]}

		ConnectionInfo ret;
		ret.id = in[0];
		ret.destination = static_cast<std::string>(in[1]);

		std::string d = in[2];
		if(d == "i")
			ret.direction = Direction::In;
		else if(d == "o")
			ret.direction = Direction::Out;
		else if(d == "b")
			ret.direction = Direction::Both;
		else
			throw std::invalid_argument("Unknown direction string");

		ret.transport = static_cast<std::string>(in[3]);
		ret.topic = static_cast<std::string>(in[4]);

		return ret;
	}

	void updateBytes(int32_t newBytes, const ros::WallDuration& dt)
	{
		if(bytesValid)
		{
			int32_t diff = newBytes - bytes;
			bitsPerSecond = static_cast<uint64_t>(diff) * 8 / dt.toSec();
		}

		bytes = newBytes;
		bytesValid = true;
	}
};

struct NodeInfo
{
	std::string name;
	std::string uri;
	std::string host;
	uint32_t port;

	std::map<std::string, ConnectionInfo> connections;
	bool active = false;

	class LookupException : std::runtime_error
	{
	public:
		LookupException(const char* what) : std::runtime_error(what) {}
	};

	static NodeInfo fromName(const std::string& name)
	{
		NodeInfo ret;

		XmlRpc::XmlRpcValue n_response;
		XmlRpc::XmlRpcValue n_payload;

		XmlRpc::XmlRpcValue args;
		args[0] = "net_monitor";
		args[1] = name;

		if(!ros::master::execute("lookupNode", args, n_response, n_payload, false))
		{
			ROS_WARN("Could not lookup node '%s'", name.c_str());
			throw LookupException("Could not find the node");
		}

		ret.name = name;
		ret.uri = static_cast<std::string>(n_payload);

		if(!ros::network::splitURI(ret.uri, ret.host, ret.port))
		{
			ROS_WARN("Could not parse node URI: '%s'", ret.uri.c_str());
			throw LookupException("Could not parse node URI");
		}

		return ret;
	}
};

struct Peer
{
	std::map<std::string, NodeInfo> nodes;
	bool active = false;
};

struct Interface
{
	Interface(const std::string& name)
	 : name(name)
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

		updateStats();
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

	void updateStats()
	{
		ros::SteadyTime now = ros::SteadyTime::now();

		auto newRX = readProp("statistics/rx_bytes");
		auto newTX = readProp("statistics/tx_bytes");

		if(!newRX || !newTX)
			return;

		double timeDelta = (now - lastStatsTime).toSec();
		rx_bandwidth = 8 * ((*newRX) - rx_bytes) / timeDelta;
		tx_bandwidth = 8 * ((*newTX) - tx_bytes) / timeDelta;

		rx_bytes = *newRX;
		tx_bytes = *newTX;
		lastStatsTime = now;
	}

	std::string name;
	uint64_t bitsPerSecond = 0;
	bool duplex = false;
	std::map<std::string, Peer> peers;

	uint64_t rx_bytes = 0;
	uint64_t tx_bytes = 0;

	double rx_bandwidth = 0;
	double tx_bandwidth = 0;

	ros::SteadyTime lastStatsTime;
};

class NetMonitor
{
public:
	typedef std::map<std::string, Peer> PeerMap;

	NetMonitor()
	{
		ros::NodeHandle nh("~");

		double nodePeriod;
		nh.param("node_period", nodePeriod, 4.0);

		m_nodeTimer = nh.createSteadyTimer(
			ros::WallDuration(nodePeriod),
			std::bind(&NetMonitor::updateNodes, this)
		);

		double statsPeriod;
		nh.param("stats_period", statsPeriod, 2.0);

		m_statsTimer = nh.createSteadyTimer(
			ros::WallDuration(statsPeriod),
			std::bind(&NetMonitor::updateStats, this)
		);

		m_pub = nh.advertise<nimbro_net_monitor::NetworkStats>("/network/monitor", 1, true);

		updateNodes();
		updateStats();
	}

	void updateNodes()
	{
		XmlRpc::XmlRpcValue response;
		XmlRpc::XmlRpcValue payload;
		if(!ros::master::execute("getSystemState", "net_monitor", response, payload, false))
		{
			ROS_ERROR("Could not query system state from master");
			return;
		}

		std::set<std::string> nodeNames;

		for(int i = 0; i < payload.size(); ++i)
		{
			auto advertisement = payload[i];

			for(int j = 0; j < advertisement.size(); ++j)
			{
				auto item = advertisement[j];

				auto pubs = item[1];

				for(int k = 0; k < pubs.size(); ++k)
					nodeNames.insert(pubs[k]);
			}
		}

		m_nodesByURI.clear();
		m_nodesByName.clear();

		for(auto& name : nodeNames)
		{
			try
			{
				auto n = std::make_shared<NodeInfo>(NodeInfo::fromName(name));
				m_nodesByURI[n->uri] = n;
				m_nodesByName[n->name] = n;
			}
			catch(NodeInfo::LookupException&)
			{
				continue;
			}
		}
	}

	void updateConnectionStats(const ConnectionInfo& connection, int32_t bytes, const ros::WallDuration& dt)
	{
		// For publications, we get a node *name*, for subscriptions, we get a
		// XMLRPC URI. *sigh*
		auto nodeIt = m_nodesByURI.find(connection.destination);
		if(nodeIt == m_nodesByURI.end())
		{
			nodeIt = m_nodesByName.find(connection.destination);
			if(nodeIt == m_nodesByName.end())
			{
				ROS_WARN("Could not find node '%s' in local cache", connection.destination.c_str());
				return;
			}
		}

		bool local = (nodeIt->second->host == ros::network::getHost());

		if(local)
			return;

		std::string interfaceName = m_routeCache.obtainInterfaceForHost(nodeIt->second->host);
		if(interfaceName.empty())
			interfaceName = "unknown";

		auto ifaceIt = m_interfaces.find(interfaceName);
		if(ifaceIt == m_interfaces.end())
		{
			Interface interface(interfaceName);
			ifaceIt = m_interfaces.insert(std::make_pair(interfaceName, interface)).first;
		}

		Interface& interface = ifaceIt->second;

		Peer& peer = interface.peers[nodeIt->second->host];
		peer.active = true;

		auto& nodeStats = peer.nodes[nodeIt->second->name];
		nodeStats.active = true;

		auto hash = connection.hash();
		auto conIt = nodeStats.connections.find(hash);
		if(conIt != nodeStats.connections.end())
		{
			auto& cacheCon = conIt->second;
			if(cacheCon.destination != connection.destination
				|| cacheCon.topic != connection.topic
				|| cacheCon.transport != connection.transport)
			{
				ROS_WARN("connection does not match cached connection, strange.");
				nodeStats.connections.erase(conIt);
				conIt = nodeStats.connections.end();
			}
		}

		if(conIt == nodeStats.connections.end())
		{
			conIt = nodeStats.connections.insert(
				std::make_pair(hash, connection)
			).first;
		}

		auto& cacheCon = conIt->second;

		if(cacheCon.active)
		{
			ROS_WARN("I got two stat responses for one connection, ignoring...");
			return;
		}

		cacheCon.updateBytes(bytes, dt);
		cacheCon.active = true;
	}

	void updateNodeStats(const NodeInfo& nodeInfo, const ros::WallDuration& dt)
	{
		std::string name = nodeInfo.name;

		// Let's call the node for information!
		XmlRpc::XmlRpcValue stats_response;

		if(!slaveCall(nodeInfo.host, nodeInfo.port, "getBusStats", "net_monitor", stats_response))
		{
			// This happens all the time since the roscore has many stale node infos
			ROS_DEBUG("Could not call node '%s'", name.c_str());
			return;
		}

		if(stats_response.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_WARN("Ignoring '%s'", name.c_str());
			return;
		}

		// python nodes report (success code, status, response)
		// C++ nodes report response directly. go figure.
		XmlRpc::XmlRpcValue publish_stats;
		XmlRpc::XmlRpcValue subscribe_stats;
		if(stats_response[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
		{
			publish_stats = stats_response[2][0];
			subscribe_stats = stats_response[2][1];
		}
		else
		{
			publish_stats = stats_response[0];
			subscribe_stats = stats_response[1];
		}

		XmlRpc::XmlRpcValue info_response;
		if(!slaveCall(nodeInfo.host, nodeInfo.port, "getBusInfo", "net_monitor", info_response))
		{
			ROS_WARN("Could not call node '%s'", name.c_str());
			return;
		}

		auto info_array = info_response[2];

		std::map<int, ConnectionInfo> connections;
		for(int i = 0; i < info_array.size(); ++i)
		{
			auto c = ConnectionInfo::fromXmlRpc(info_array[i]);
// 			ROS_INFO("Node '%s' at '%s': connection to '%s' on topic '%s'",
// 				nodeInfo.name.c_str(), nodeInfo.uri.c_str(),
// 				c.destination.c_str(), c.topic.c_str()
// 			);
			c.localNode = nodeInfo.name;
			connections[c.id] = c;
		}

		for(int i = 0; i < publish_stats.size(); ++i)
		{
			auto publication = publish_stats[i];

			// NOTE: Again, python nodes have (topic, msg num, links),
			// C++ nodes have (topic, links)...
			auto links = publication[publication.size()-1];

			for(int j = 0; j < links.size(); ++j)
			{
				auto link = links[j];
				int conId = link[0];

				auto it = connections.find(conId);
				if(it == connections.end())
				{
					ROS_WARN("Could not find connection");
					continue;
				}

				auto& connection = it->second;

				updateConnectionStats(connection, static_cast<int>(link[1]), dt);
			}
		}

		for(int i = 0; i < subscribe_stats.size(); ++i)
		{
			auto publication = subscribe_stats[i];

			// NOTE: Again, python nodes have (topic, msg num, links),
			// C++ nodes have (topic, links)...
			auto links = publication[publication.size()-1];

			for(int j = 0; j < links.size(); ++j)
			{
				auto link = links[j];
				int conId = link[0];

				auto it = connections.find(conId);
				if(it == connections.end())
				{
					ROS_WARN("Could not find connection");
					continue;
				}

				auto& connection = it->second;

				updateConnectionStats(connection, static_cast<int>(link[1]), dt);
			}
		}
	}

	void updateStats()
	{
		ros::SteadyTime now = ros::SteadyTime::now();

		ros::WallDuration dt = now - m_lastTime;

		for(auto& iface : m_interfaces)
		{
			iface.second.updateStats();

			for(auto& peer : iface.second.peers)
			{
				peer.second.active = false;
				for(auto& node : peer.second.nodes)
				{
					node.second.active = false;
					for(auto& con : node.second.connections)
					{
						con.second.active = false;
					}
				}
			}
		}

		for(auto& pair : m_nodesByURI)
		{
			auto& node = pair.second;

			if(node->host != ros::network::getHost())
				continue;

// 			ROS_INFO(" - %s at %s", node->name.c_str(), node->uri.c_str());

			updateNodeStats(*node, dt);
		}

		// Prune old information
		for(auto ifaceIt = m_interfaces.begin(); ifaceIt != m_interfaces.end(); ++ifaceIt)
		{
			auto& peers = ifaceIt->second.peers;
			for(auto it = peers.begin(); it != peers.end();)
			{
				if(it->second.active)
				{
					auto& peer = it->second;

					for(auto nodeIt = peer.nodes.begin(); nodeIt != peer.nodes.end();)
					{
						if(nodeIt->second.active)
						{
							auto& node = nodeIt->second;

							for(auto conIt = node.connections.begin(); conIt != node.connections.end();)
							{
								if(conIt->second.active)
									++conIt;
								else
									conIt = node.connections.erase(conIt);
							}

							++nodeIt;
						}
						else
							nodeIt = peer.nodes.erase(nodeIt);
					}

					++it;
				}
				else
					it = peers.erase(it);
			}
		}

		// Generate message
		nimbro_net_monitor::NetworkStats stats;
		stats.header.stamp = ros::Time::now();

		{
			char buf[256];
			gethostname(buf, sizeof(buf));
			buf[sizeof(buf)-1] = 0;
			stats.host = buf;
		}

		for(auto& ifacePair : m_interfaces)
		{
			auto& iface = ifacePair.second;

			nimbro_net_monitor::InterfaceStats ifaceStats;
			ifaceStats.interface_name = iface.name;
			ifaceStats.max_bits_per_second = iface.bitsPerSecond;
			ifaceStats.duplex = iface.duplex;
			ifaceStats.rx_bandwidth = iface.rx_bandwidth;
			ifaceStats.tx_bandwidth = iface.tx_bandwidth;

			for(auto& peerIt : ifacePair.second.peers)
			{
				const auto& peer = peerIt.second;

				nimbro_net_monitor::PeerStats peerStats;

				peerStats.host = peerIt.first;

				for(auto& nodeIt : peer.nodes)
				{
					nimbro_net_monitor::NodeStats nodeStats;

					nodeStats.name = nodeIt.first;

					for(auto& conIt : nodeIt.second.connections)
					{
						const auto& con = conIt.second;

						nimbro_net_monitor::ConnectionStats conStats;

						conStats.topic = con.topic;
						conStats.destination = con.destination;

						switch(con.direction)
						{
							case ConnectionInfo::Direction::In:
								conStats.direction = conStats.DIR_IN;
								break;
							case ConnectionInfo::Direction::Out:
								conStats.direction = conStats.DIR_OUT;
								break;
							default:
								break;
						}

						conStats.transport = con.transport;
						conStats.bits_per_second = con.bitsPerSecond;
						conStats.local_node = con.localNode;

						nodeStats.connections.push_back(std::move(conStats));
					}

					peerStats.nodes.push_back(std::move(nodeStats));
				}

				ifaceStats.peers.push_back(std::move(peerStats));
			}

			stats.interfaces.push_back(std::move(ifaceStats));
		}

		m_pub.publish(stats);

		m_lastTime = now;
	}
private:
	std::map<std::string, std::shared_ptr<NodeInfo>> m_nodesByURI;
	std::map<std::string, std::shared_ptr<NodeInfo>> m_nodesByName;

	std::map<std::string, Interface> m_interfaces;

	ros::SteadyTime m_lastTime;
	ros::SteadyTimer m_nodeTimer;
	ros::SteadyTimer m_statsTimer;

	ros::Publisher m_pub;

	route::Cache m_routeCache;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "net_monitor");

	NetMonitor monitor;

	ros::NodeHandle nh("~");


	ros::spin();

	return 0;
}
