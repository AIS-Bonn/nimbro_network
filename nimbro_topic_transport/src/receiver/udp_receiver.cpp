// Receive UDP packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_receiver.h"

#include <ros/node_handle.h>
#include <ros/console.h>

#include <errno.h>
#include <stdio.h>
#include <netdb.h>

#include <fstream>

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

	int logBufferSize = 0;
	if(nh.getParam("log_buffer_size", logBufferSize))
	{
		if(logBufferSize < 0)
			throw std::runtime_error{"Invalid buffer size"};

		m_logBuffer.resize(logBufferSize);

		m_srv_dumpLog = nh.advertiseService("dump_log", &UDPReceiver::dumpLog, this);
	}
}

UDPReceiver::~UDPReceiver()
{
	m_shouldExit = true;
	m_thread.join();
}

void UDPReceiver::start()
{
	if(!m_callback)
		throw std::logic_error{"Call setCallback() before start()!"};

	m_thread = std::thread(std::bind(&UDPReceiver::thread, this));
}

void UDPReceiver::thread()
{
	pthread_setname_np(pthread_self(), "udp_rx");

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

		sockaddr_storage addr{};
		socklen_t addrlen = sizeof(addr);

		ssize_t size = recvfrom(m_fd, packet->data.data(), packet->data.size(), 0, (sockaddr*)&addr, &addrlen);

		if(size < 0)
		{
			ROS_FATAL("Could not recv(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		ROS_DEBUG_NAMED("udp", "Received UDP packet of size %ld", size);
		packet->length = size;
		packet->srcReceiveTime = ros::Time::now();

		m_callback(packet);

		if(!m_logBuffer.empty())
		{
			std::unique_lock<std::mutex> lock{m_logMutex};

			std::uint16_t messageID = -1;
			std::uint32_t packetID = -1;

			if(packet->length >= sizeof(UDPPacket::Header))
			{
				messageID = packet->packet()->header.msg_id;
				packetID = packet->packet()->header.symbol_id;
			}

			if(m_logBufferCount != m_logBuffer.size())
				m_logBuffer[m_logBufferCount++] = {messageID, packetID, packet->srcReceiveTime};
			else
			{
				m_logBuffer[m_logBufferOffset] = {messageID, packetID, packet->srcReceiveTime};
				m_logBufferOffset = (m_logBufferOffset + 1) % m_logBuffer.size();
			}
		}
	}
}

void UDPReceiver::setCallback(const Callback& cb)
{
	m_callback = cb;
}

bool UDPReceiver::dumpLog(DumpLogRequest& req, DumpLogResponse& resp)
{
	std::unique_lock<std::mutex> lock{m_logMutex};

	std::ofstream out(req.destination_path);

	out << "Stamp MessageID SymbolID\n";

	std::size_t size = m_logBuffer.size();
	for(std::size_t i = 0; i < m_logBufferCount; ++i)
	{
		auto& entry = m_logBuffer[(i + m_logBufferOffset) % size];

		out << entry.receiptTime.toSec() << " " << entry.messageID << " " << entry.symbolID << "\n";
	}

	return true;
}

}
