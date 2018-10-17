// ROS component
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>
#include <ros/master.h>
#include <ros/network.h>

#include "ros/xmlrpc_manager.h"
#include "xmlrpcpp/XmlRpc.h"

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
	bool connected = false;

	static ConnectionInfo fromXmlRpc(XmlRpc::XmlRpcValue& in)
	{
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
		ret.connected = static_cast<int>(in[5]);

		return ret;
	}

	bool isLocal() const
	{
		return destination == ros::network::getHost(); // FIXME: destination is node *name*
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "net_monitor");

	XmlRpc::XmlRpcValue response;
	XmlRpc::XmlRpcValue payload;
	if(!ros::master::execute("getSystemState", "net_monitor", response, payload, false))
	{
		ROS_ERROR("Could not query system state from master");
		return 1;
	}

	std::set<std::string> nodes;

	for(int i = 0; i < payload.size(); ++i)
	{
		auto advertisement = payload[i];

		for(int j = 0; j < advertisement.size(); ++j)
		{
			auto item = advertisement[j];

			auto pubs = item[1];

			for(int k = 0; k < pubs.size(); ++k)
				nodes.insert(pubs[k]);
		}
	}

	for(auto& node : nodes)
	{
		XmlRpc::XmlRpcValue n_response;
		XmlRpc::XmlRpcValue n_payload;

		XmlRpc::XmlRpcValue args;
		args[0] = "net_monitor";
		args[1] = node;

		if(!ros::master::execute("lookupNode", args, n_response, n_payload, false))
		{
			ROS_WARN("Could not lookup node '%s'", node.c_str());
			continue;
		}

		std::string uri = n_payload;
		std::string host;
		uint32_t port;
		if(!ros::network::splitURI(uri, host, port))
		{
			ROS_WARN("Could not split URI: %s", uri.c_str());
			continue;
		}

		if(host != ros::network::getHost())
			continue;

		ROS_INFO(" - %s at %s", node.c_str(), uri.c_str());

		// Let's call the node for information!

		XmlRpc::XmlRpcValue stats_response;

		if(!slaveCall(host, port, "getBusStats", "net_monitor", stats_response))
		{
			ROS_WARN("Could not call node '%s'", node.c_str());
			continue;
		}

		auto publish_stats = stats_response[0];
		auto subscribe_stats = stats_response[1];

		XmlRpc::XmlRpcValue info_response;
		if(!slaveCall(host, port, "getBusInfo", "net_monitor", info_response))
		{
			ROS_WARN("Could not call node '%s'", node.c_str());
			continue;
		}

		auto info_array = info_response[2];
		ROS_INFO_STREAM("info: " << info_response);

		std::map<int, ConnectionInfo> connections;
		for(int i = 0; i < info_array.size(); ++i)
		{
			auto c = ConnectionInfo::fromXmlRpc(info_array[i]);
			connections[c.id] = c;
		}

		ROS_INFO("%lu connections", connections.size());

		for(int i = 0; i < publish_stats.size(); ++i)
		{
			auto publication = publish_stats[i];
			ROS_INFO("  - Topic '%s':", static_cast<std::string>(publication[0]).c_str());

			auto links = publication[1];

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

				ROS_INFO("    - connection to '%s' (%s): %d bytes",
					connection.destination.c_str(),
					connection.isLocal() ? "local" : "remote",
					static_cast<int>(link[1])
				);
			}
		}
	}

	return 0;
}
