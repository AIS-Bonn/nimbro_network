// Send messages over TCP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_sender.h"
#include "../topic_info.h"
#include "../tcp_packet.h"

#include <netinet/tcp.h>
#include <netdb.h>
#include <sys/uio.h>

namespace nimbro_topic_transport
{

TCPSender::TCPSender(ros::NodeHandle& nh)
 : m_nh{nh}
 , m_fd(-1)
{
	std::string addr;
	if(!m_nh.getParam("destination_addr", addr) && !m_nh.getParam("address", addr))
	{
		ROS_FATAL("tcp_sender needs an 'destination_addr' parameter!");
		throw std::runtime_error("tcp_sender needs an 'destination_addr' parameter!");
	}

	int port;
	if(!m_nh.getParam("tcp/port", port) && !m_nh.getParam("port", port))
	{
		ROS_FATAL("tcp_sender needs a 'destination_port' parameter!");
		throw std::runtime_error("tcp_sender needs a 'destination_port' parameter!");
	}

	std::string portStr = boost::lexical_cast<std::string>(port);

	if(m_nh.hasParam("tcp/source_port") || m_nh.hasParam("source_port"))
	{
		if(!m_nh.getParam("tcp/source_port", m_sourcePort) && !m_nh.getParam("source_port", m_sourcePort))
		{
			ROS_FATAL("Invalid source_port");
			throw std::runtime_error("Invalid source port");
		}
	}
	else
		m_sourcePort = -1;

	addrinfo* info = 0;
	if(getaddrinfo(addr.c_str(), portStr.c_str(), 0, &info) != 0 || !info)
	{
		ROS_FATAL("Could not lookup host name '%s'", addr.c_str());
		throw std::runtime_error("Could not lookup hostname");
	}

	m_addrFamily = info->ai_family;
	memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
	m_addrLen = info->ai_addrlen;

	freeaddrinfo(info);
}

TCPSender::~TCPSender()
{
}

bool TCPSender::connect()
{
	m_fd = socket(m_addrFamily, SOCK_STREAM, 0);
	if(m_fd < 0)
	{
		ROS_ERROR("Could not create socket: %s", strerror(errno));
		return false;
	}

	if(m_sourcePort != -1)
	{
		std::string source_port_str = boost::lexical_cast<std::string>(m_sourcePort);

		addrinfo hints;
		memset(&hints, 0, sizeof(hints));

		hints.ai_flags = AI_PASSIVE;
		hints.ai_family = m_addrFamily;
		hints.ai_socktype = SOCK_STREAM;

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

	if(::connect(m_fd, (sockaddr*)&m_addr, m_addrLen) != 0)
	{
		ROS_ERROR_THROTTLE(5.0, "Could not connect: %s", strerror(errno));
		return false;
	}
	ROS_INFO("Connected to destination");

	if(m_sourcePort == -1)
	{
		sockaddr_storage addr;
		socklen_t addrlen = sizeof(addr);

		char nameBuf[256];
		char serviceBuf[256];

		if(getsockname(m_fd, (sockaddr*)&addr, &addrlen) != 0)
		{
			ROS_ERROR("Could not get local socket addr: %s", strerror(errno));
		}

		if(getnameinfo((sockaddr*)&addr, addrlen, nameBuf, sizeof(nameBuf), serviceBuf, sizeof(serviceBuf), NI_NUMERICSERV | NI_NUMERICHOST) != 0)
		{
			ROS_ERROR("Could not resolve remote address to name");
		}
	}

#ifdef TCP_USER_TIMEOUT
	int timeout = 5000;
	if(setsockopt(m_fd, SOL_TCP, TCP_USER_TIMEOUT, &timeout, sizeof(timeout)) != 0)
	{
		ROS_ERROR("Could not set TCP_USER_TIMEOUT: %s", strerror(errno));
		return false;
	}
#else
	ROS_WARN("Not setting TCP_USER_TIMEOUT");
#endif

	return true;
}

void TCPSender::send(const Message::ConstPtr& msg)
{
	// NOTE: Unprotected part, do not access members here
	std::size_t headerSize =
		sizeof(TCPHeader) + msg->topic->name.length() + msg->type.length();

	std::vector<uint8_t> headerStorage(headerSize);
	TCPHeader* header = reinterpret_cast<TCPHeader*>(headerStorage.data());

	memcpy(
		headerStorage.data() + sizeof(TCPHeader),
		msg->topic->name.c_str(), msg->topic->name.length()
	);
	memcpy(
		headerStorage.data() + sizeof(TCPHeader) + msg->topic->name.length(),
		msg->type.c_str(), msg->type.length()
	);

	header->data_len = msg->payload.size();
	header->type_len = msg->type.length();
	header->topic_len = msg->topic->name.length();
	header->flags = msg->flags;
	topic_info::packMD5(msg->md5, header->topic_md5sum);

	msghdr socket_msg;
	memset(&socket_msg, 0, sizeof(socket_msg));

	std::vector<iovec> iovecs(2);
	iovecs[0].iov_base = headerStorage.data();
	iovecs[0].iov_len = headerStorage.size();
	iovecs[1].iov_base = const_cast<uint8_t*>(msg->payload.data());
	iovecs[1].iov_len = msg->payload.size();

	std::size_t totalSize = headerStorage.size() + msg->payload.size();

	// Lock and send
	// NOTE: Protected part starts here
	std::unique_lock<std::mutex> lock(m_mutex);

	// Try to send the packet
	for(int tries = 0; tries < 10; ++tries)
	{
		if(m_fd == -1)
		{
			if(!connect())
			{
				ROS_WARN_THROTTLE(1.0, "Connection failed, trying again");
				continue;
			}
		}

		if(writev(m_fd, iovecs.data(), iovecs.size()) != (int)totalSize)
		{
			ROS_WARN_THROTTLE(1.0, "Could not send data, trying again");
			close(m_fd);
			m_fd = -1;
			continue;
		}

		// Read ACK
		uint8_t ack;
		if(read(m_fd, &ack, 1) != 1)
		{
			ROS_WARN("Could not read ACK, sending again (!)");
			close(m_fd);
			m_fd = -1;
			continue;
		}

		return;
	}

	ROS_ERROR_THROTTLE(5.0,
		"Could not send TCP packet. Dropping message from topic %s!",
		msg->topic->name.c_str()
	);
}

}
