// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"

#include <netdb.h>

namespace nimbro_topic_transport
{

UDPSender::UDPSender()
{
	ros::NodeHandle nh("~");

	// Get ROS parameters
	std::string dest_host;
	nh.param("destination_addr", dest_host, std::string("localhost"));

	int dest_port;
	if(!nh.getParam("udp/port", dest_port))
		nh.param("port", dest_port, 5050);

	std::string dest_port_str = boost::lexical_cast<std::string>(dest_port);

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

	m_fd = socket(info->ai_family, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		ROS_FATAL("Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
	{
		ROS_FATAL("Could not enable SO_BROADCAST flag: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
	m_addrLen = info->ai_addrlen;

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

		if(bind(m_fd, localInfo->ai_addr, localInfo->ai_addrlen) != 0)
		{
			ROS_FATAL("Could not bind to source port: %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		freeaddrinfo(localInfo);
	}

	freeaddrinfo(info);
}

UDPSender::~UDPSender()
{
}

void UDPSender::send(const std::vector<Packet::Ptr>& packets)
{
	for(auto& packet : packets)
	{
		ROS_DEBUG_NAMED("udp", "Sending UDP packet of size %lu", packet->data.size());
		if(sendto(m_fd, packet->data.data(), packet->data.size(), 0, (sockaddr*)&m_addr, m_addrLen) != (ssize_t)packet->data.size())
		{
			ROS_ERROR("Could not send data of size %d: %s", (int)packet->data.size(), strerror(errno));
			return;
		}
	}
}

}
