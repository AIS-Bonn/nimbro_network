// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"

#include <netdb.h>
#include <sys/uio.h>

namespace nimbro_topic_transport
{

UDPSender::UDPSender(ros::NodeHandle& nh)
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


	int dest_port;
	if(!nh.getParam("udp/port", dest_port))
		nh.param("port", dest_port, 5050);

	std::string dest_port_str = std::to_string(dest_port);

	for(auto& dest_host : destination_addrs)
	{
		auto& sock = m_sockets.emplace_back();

		// Resolve the destination address
		// note: getaddrinfo() also accepts direct IP addresses
		addrinfo *info = 0;
		if(getaddrinfo(dest_host.c_str(), dest_port_str.c_str(), 0, &info) != 0 || !info)
		{
			ROS_FATAL("Could not lookup destination address\n '%s': %s",
				dest_host.c_str(), strerror(errno)
			);
			throw std::runtime_error(strerror(errno));
		}

		sock.fd = socket(info->ai_family, SOCK_DGRAM, 0);
		if(sock.fd < 0)
		{
			ROS_FATAL("Could not create socket: %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		int on = 1;
		if(setsockopt(sock.fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
		{
			ROS_FATAL("Could not enable SO_BROADCAST flag: %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		memcpy(&sock.addr, info->ai_addr, info->ai_addrlen);
		sock.addrLen = info->ai_addrlen;

		int source_port = dest_port;

		// If we have a specified source port, bind to it.
		if(nh.hasParam("udp/source_port") || nh.hasParam("source_port"))
		{
			if(!nh.getParam("udp/source_port", source_port) && !nh.getParam("source_port", source_port))
			{
				ROS_FATAL("Invalid source_port");
				throw std::runtime_error("Invalid source port");
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
				throw std::runtime_error("Could not get local address");
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
				throw std::runtime_error(strerror(errno));
			}

			freeaddrinfo(localInfo);
		}

		freeaddrinfo(info);
	}

	m_statTimer = nh.createSteadyTimer(ros::WallDuration(5.0), std::bind(&UDPSender::printStats, this));
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

		delay += ros::Time::now() - packet->srcReceiveTime;
	}

	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_statPackets += packets.size();
		m_statDelay += delay;
	}
}

void UDPSender::printStats()
{
	uint64_t packets = 0;
	ros::Duration delay;
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		std::swap(packets, m_statPackets);
		std::swap(delay, m_statDelay);
	}

	ROS_INFO_NAMED("udp", "UDP sender: %.2f packets per sec, %.2fms delay introduced by sender",
		packets / 5.0,
		delay.toSec() / 5.0 / packets * 1000.0
	);
}

}
