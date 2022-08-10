// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"

#include <nimbro_topic_transport/SenderStats.h>

#include <netdb.h>
#include <sys/uio.h>

namespace nimbro_topic_transport
{

UDPSender::UDPSender(ros::NodeHandle& nh)
 : m_nh{nh}
{
	// Get ROS parameters
	XmlRpc::XmlRpcValue param_dest;
	if(!nh.getParam("destination_addr", param_dest))
	{
		ROS_FATAL("I need a destination_addr parameter!");
		std::exit(1);
	}

	std::vector<std::string> destination_addrs;

	if(param_dest.getType() == XmlRpc::XmlRpcValue::TypeString)
		destination_addrs.push_back(static_cast<std::string>(param_dest));
	else if(param_dest.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		for(int i = 0; i < param_dest.size(); ++i)
		{
			auto val = param_dest[i];
			if(val.getType() != XmlRpc::XmlRpcValue::TypeString)
			{
				ROS_FATAL("destination_addr should be a list of strings");
				std::exit(1);
			}

			destination_addrs.push_back(static_cast<std::string>(val));
		}
	}
	else
	{
		ROS_FATAL("I could not understand the destination_addr parameter");
		std::exit(1);
	}


	if(!nh.getParam("udp/port", m_destinationPort))
		nh.param("port", m_destinationPort, 5050);

	if(!setupSockets(destination_addrs))
	{
		ROS_FATAL("Could not create sockets to destination");
		std::exit(1);
	}

	char buf[256];
	if(gethostname(buf, sizeof(buf)) != 0)
	{
		ROS_FATAL("Could not get hostname");
		std::exit(1);
	}
	m_hostname = buf;

	m_pub_stats = nh.advertise<nimbro_topic_transport::SenderStats>("udp/stats", 1);

	m_statTimer = nh.createSteadyTimer(ros::WallDuration(0.1), std::bind(&UDPSender::sendStats, this));

	m_srv_setDestinations = nh.advertiseService(
		"udp/set_destinations", &UDPSender::handleSetDestinations, this
	);
}

UDPSender::~UDPSender()
{
}

void UDPSender::send(const std::vector<Packet::Ptr>& packets)
{
	// Reserve range of packet IDs. As long as our messages do not
	// approach 2^24 bytes (~25GB), this is fine.
	uint32_t packetID;
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		packetID = m_packetID;
		m_packetID += packets.size();
	}

	ros::Duration delay;

	for(auto& packet : packets)
	{
		// Patch the packet ID
		packet->packet()->header.packet_id = packetID++;

		ROS_DEBUG_NAMED("udp", "Sending UDP packet of size %lu", packet->length);
		for(auto& sock : m_sockets)
		{
			if(sendto(sock.fd, packet->data.data(), packet->length, 0, (sockaddr*)&sock.addr, sock.addrLen) != (ssize_t)packet->length)
			{
				ROS_ERROR("Could not send data of size %d: %s", (int)packet->length, strerror(errno));
				return;
			}
		}

		ros::Time now = ros::Time::now();
		if(packet->srcReceiveTime != ros::Time(0) && now > packet->srcReceiveTime)
			delay += now - packet->srcReceiveTime;
	}

	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_statPackets += packets.size();
		m_statDelay += delay;

		for(auto& packet : packets)
		{
			if(packet->topic)
			{
				m_topicBandwidth[packet->topic->name] += packet->length;
			}
		}
	}
}

void UDPSender::sendStats()
{
	ros::Time now = ros::Time::now();
	double deltaTime = (now - m_lastStatTime).toSec();

	uint64_t packets = 0;
	ros::Duration delay;
	std::map<std::string, std::uint64_t> topicBandwidth;
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		std::swap(packets, m_statPackets);
		std::swap(delay, m_statDelay);
		topicBandwidth = m_topicBandwidth;
	}

	std::uint64_t totalBandwidthBytes = 0;
	for(auto& pair : topicBandwidth)
		totalBandwidthBytes += pair.second;

	nimbro_topic_transport::SenderStats msg;
	msg.header.stamp = ros::Time::now();
	msg.host = m_hostname;

	for(auto& sock : m_sockets)
	{
		auto& m = msg.destinations.emplace_back();
		m.destination = sock.destination;
		m.destination_port = m_destinationPort;
		m.source_port = sock.source_port;
	}

	msg.label = m_label;
	msg.node = ros::this_node::getName();
	msg.protocol = "UDP";
	msg.bandwidth = totalBandwidthBytes * 8 / deltaTime;

	for(auto& pair : topicBandwidth)
	{
		auto& m = msg.topics.emplace_back();
		m.name = pair.first;
		m.bandwidth = pair.second * 8 / deltaTime;
	}

	m_pub_stats.publish(msg);

	m_lastStatTime = now;

	for(auto& pair : topicBandwidth)
		pair.second = 0;

	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_topicBandwidth = topicBandwidth;
	}
}

bool UDPSender::setupSockets(const std::vector<std::string>& destination_addrs)
{
	std::string dest_port_str = std::to_string(m_destinationPort);

	m_sockets.clear();

	for(auto& dest_host : destination_addrs)
	{
		auto& sock = m_sockets.emplace_back();

		sock.destination = dest_host;

		// Resolve the destination address
		// note: getaddrinfo() also accepts direct IP addresses
		addrinfo *info = 0;
		if(getaddrinfo(dest_host.c_str(), dest_port_str.c_str(), 0, &info) != 0 || !info)
		{
			ROS_FATAL("Could not lookup destination address\n '%s': %s",
				dest_host.c_str(), strerror(errno)
			);
			return false;
		}

		sock.fd = socket(info->ai_family, SOCK_DGRAM, 0);
		if(sock.fd < 0)
		{
			ROS_FATAL("Could not create socket: %s", strerror(errno));
			return false;
		}

		int on = 1;
		if(setsockopt(sock.fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
		{
			ROS_FATAL("Could not enable SO_BROADCAST flag: %s", strerror(errno));
			return false;
		}

		memcpy(&sock.addr, info->ai_addr, info->ai_addrlen);
		sock.addrLen = info->ai_addrlen;

		int source_port = 0;

		// If we have a specified source port, bind to it.
		if(m_nh.hasParam("udp/source_port") || m_nh.hasParam("source_port"))
		{
			if(!m_nh.getParam("udp/source_port", source_port) && !m_nh.getParam("source_port", source_port))
			{
				ROS_FATAL("Invalid source_port");
				return false;
			}
		}

		std::string source_port_str = boost::lexical_cast<std::string>(source_port);

		addrinfo hints;
		memset(&hints, 0, sizeof(hints));

		hints.ai_flags = AI_PASSIVE;
		hints.ai_family = info->ai_family;
		hints.ai_socktype = SOCK_DGRAM;

		addrinfo* localInfo = 0;
		if(getaddrinfo(NULL, source_port_str.c_str(), &hints, &localInfo) != 0 || !localInfo)
		{
			ROS_FATAL("Could not get local address: %s", strerror(errno));
			return false;
		}

		// If we have multiple destination addrs, set SO_REUSEPORT so that we can share
		// the same source port
		if(destination_addrs.size() > 1)
		{
			int on = 1;
			if(setsockopt(sock.fd, SOL_SOCKET, SO_REUSEPORT, &on, sizeof(on)) != 0)
			{
				ROS_WARN("Could not set SO_REUSEPORT: %s", strerror(errno));
			}
		}

		if(bind(sock.fd, localInfo->ai_addr, localInfo->ai_addrlen) != 0)
		{
			ROS_FATAL("Could not bind to source port: %s", strerror(errno));
			return false;
		}

		freeaddrinfo(localInfo);

		// Find out which port we bound to
		sockaddr_storage addr;
		socklen_t addrLen = sizeof(addr);
		if(getsockname(sock.fd, reinterpret_cast<sockaddr*>(&addr), &addrLen) == 0)
		{
			if(addr.ss_family == AF_INET)
				sock.source_port = ntohs(reinterpret_cast<sockaddr_in*>(&addr)->sin_port);
			else if(addr.ss_family == AF_INET6)
				sock.source_port = ntohs(reinterpret_cast<sockaddr_in6*>(&addr)->sin6_port);
			else
			{
				ROS_WARN("Unsupported address family");
				sock.source_port = 0;
			}
		}

		freeaddrinfo(info);
	}

	return true;
}

bool UDPSender::handleSetDestinations(SetDestinationsRequest& req, SetDestinationsResponse& resp)
{
	std::stringstream ss;
	for(auto& dest : req.destinations)
		ss << " " << dest;

	ROS_INFO("Reconnect to destinations:%s", ss.str().c_str());
	resp.success = setupSockets(req.destinations);
	return true;
}

}
