// Receive UDP packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_receiver.h"

#include <ros/node_handle.h>
#include <ros/console.h>

#include <errno.h>
#include <stdio.h>
#include <netdb.h>

namespace nimbro_topic_transport
{

UDPReceiver::UDPReceiver(ros::NodeHandle& nh)
{
	m_fd = socket(AF_INET6, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		ROS_FATAL("Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int port;
	nh.param("port", port, 5050);

	sockaddr_in6 addr;
	addr.sin6_family = AF_INET6;
	addr.sin6_addr = in6addr_any;
	addr.sin6_port = htons(port);

	ROS_INFO("UDP: Binding to :%d", port);

	if(bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		ROS_FATAL("Could not bind socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
	{
		ROS_FATAL("Could not set broadcast flag: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	m_thread = std::thread(std::bind(&UDPReceiver::thread, this));
}

UDPReceiver::~UDPReceiver()
{
	m_shouldExit = true;
	m_thread.join();
}

void UDPReceiver::thread()
{
	while(!m_shouldExit)
	{
		auto packet = std::make_shared<Packet>();

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);

		timeval timeout;
		timeout.tv_usec = 50 * 1000;
		timeout.tv_sec = 0;

		int ret = select(m_fd+1, &fds, 0, 0, &timeout);
		if(ret < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;

			ROS_FATAL("Could not select(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}
		if(ret == 0)
			continue;

		sockaddr_storage addr;
		socklen_t addrlen = sizeof(addr);

		ssize_t size = recvfrom(m_fd, packet->data.data(), packet->data.size(), 0, (sockaddr*)&addr, &addrlen);

		if(size < 0)
		{
			ROS_FATAL("Could not recv(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		if(addrlen != m_remoteAddrLen || memcmp(&addr, &m_remoteAddr, addrlen) != 0)
		{
			// Perform reverse lookup
			char nameBuf[256];
			char serviceBuf[256];

			ros::WallTime startLookup = ros::WallTime::now();
			if(getnameinfo(
				(sockaddr*)&addr, addrlen, nameBuf, sizeof(nameBuf),
				serviceBuf, sizeof(serviceBuf), NI_NUMERICSERV) == 0)
			{
				ROS_INFO("New remote: %s:%s", nameBuf, serviceBuf);
			}
			else
			{
				ROS_ERROR("Could not resolve remote address to name");
			}
			ros::WallTime endLookup = ros::WallTime::now();

			// Warn if lookup takes up time (otherwise the user does not know
			// what is going on)
			if(endLookup - startLookup > ros::WallDuration(1.0))
			{
				ROS_WARN("Reverse address lookup took more than a second. "
					"Consider adding '%s' to /etc/hosts",
					nameBuf
				);
			}

			m_remoteAddr = addr;
			m_remoteAddrLen = addrlen;
		}

		ROS_DEBUG_NAMED("udp", "Received UDP packet of size %ld", size);
		packet->length = size;
		m_callback(packet);
	}
}

void UDPReceiver::setCallback(const Callback& cb)
{
	m_callback = cb;
}

}
