// Receive TCP messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_receiver.h"

#include "../topic_info.h"

#include <netdb.h>

namespace
{
	static bool sureRead(int fd, void* dest, ssize_t size)
	{
		uint8_t* destWPtr = (uint8_t*)dest;
		fd_set fds;
		FD_ZERO(&fds);

		while(size != 0)
		{
			timeval timeout = {};
			timeout.tv_sec = 1;

			FD_SET(fd, &fds);
			auto ret = select(fd+1, &fds, nullptr, nullptr, &timeout);

			if(!ros::ok())
				return false;

			if(ret < 0)
			{
				if(errno == EAGAIN)
					continue;

				ROS_ERROR("Could not select(): %s", strerror(errno));
				return false;
			}

			// Nothing happened
			if(ret == 0)
				continue;

			ssize_t bytes = read(fd, destWPtr, size);

			if(bytes < 0)
			{
				ROS_ERROR("Could not read(): %s", strerror(errno));
				return false;
			}

			if(bytes == 0)
			{
				// Client has closed connection (ignore silently)
				return false;
			}

			size -= bytes;
			destWPtr += bytes;
		}

		return true;
	}
}

namespace nimbro_topic_transport
{

TCPReceiver::TCPReceiver(ros::NodeHandle& nh)
{
	m_fd = socket(AF_INET6, SOCK_STREAM, 0);
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

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) != 0)
	{
		ROS_FATAL("Could not enable SO_REUSEADDR: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	ROS_INFO("TCP: Binding to :%d", port);

	if(bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		ROS_FATAL("Could not bind socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	if(listen(m_fd, 10) != 0)
	{
		ROS_FATAL("Could not listen: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}
}

TCPReceiver::~TCPReceiver()
{
	m_shouldExit = true;
	m_thread.join();

	for(auto handler : m_handlers)
	{
		delete handler;
	}
}

void TCPReceiver::start()
{
	if(!m_callback)
		throw std::logic_error{"Call setCallback() before start()!"};

	m_thread = std::thread(std::bind(&TCPReceiver::thread, this));
}

void TCPReceiver::thread()
{
	fd_set fds;

	while(!m_shouldExit)
	{
		ros::spinOnce();

		// Clean up any exited threads
		std::list<ClientHandler*>::iterator it = m_handlers.begin();
		while(it != m_handlers.end())
		{
			if(!(*it)->isRunning())
			{
				delete *it;
				it = m_handlers.erase(it);
			}
			else
				it++;
		}

		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);

		timeval timeout;
		timeout.tv_usec = 0;
		timeout.tv_sec = 1;

		int ret = select(m_fd+1, &fds, 0, 0, &timeout);
		if(ret < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;

			ROS_ERROR("Could not select(): %s", strerror(errno));
			throw std::runtime_error("Could not select");
		}
		if(ret == 0)
			continue;

		sockaddr_storage remoteAddr;
		socklen_t remoteAddrLen = sizeof(remoteAddr);

		int client_fd = accept(m_fd, (sockaddr*)&remoteAddr, &remoteAddrLen);

		{
			// Perform reverse lookup
			char nameBuf[256];
			char serviceBuf[256];

			ros::WallTime startLookup = ros::WallTime::now();
			if(getnameinfo((sockaddr*)&remoteAddr, remoteAddrLen, nameBuf, sizeof(nameBuf), serviceBuf, sizeof(serviceBuf), NI_NUMERICSERV) == 0)
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
		}

		ClientHandler* handler = new ClientHandler(this, client_fd);
		m_handlers.push_back(handler);
	}
}

TCPReceiver::ClientHandler::ClientHandler(TCPReceiver* receiver, int fd)
 : m_receiver(receiver)
 , m_fd(fd)
 , m_running(true)
{
	m_thread = std::thread(std::bind(&ClientHandler::start, this));
}

TCPReceiver::ClientHandler::~ClientHandler()
{
	m_thread.join();
	close(m_fd);
}

void TCPReceiver::ClientHandler::start()
{
	m_receiver->handleClient(m_fd);
	m_running = false;
}

void TCPReceiver::handleClient(int fd)
{
	while(!m_shouldExit)
	{
		TCPHeader header;

		if(!sureRead(fd, &header, sizeof(header)))
			return;

		std::vector<char> buf(header.topic_len + 1);
		if(!sureRead(fd, buf.data(), header.topic_len))
			return;
		buf[buf.size()-1] = 0;

		std::string topic(buf.data());

		buf.resize(header.type_len+1);
		if(!sureRead(fd, buf.data(), header.type_len))
			return;
		buf[buf.size()-1] = 0;

		std::string type(buf.data());

		std::string md5;
		topic_info::unpackMD5(header.topic_md5sum, &md5);

		auto message = std::make_shared<Message>();

		std::vector<uint8_t> data(header.data_len);
		if(!sureRead(fd, data.data(), header.data_len))
			return;

		ROS_DEBUG("Got msg with flags: %d", header.flags());

		{
			std::unique_lock<std::mutex> lock(m_topicMutex);
			auto it = m_topics.find(topic);
			if(it == m_topics.end())
			{
				ROS_DEBUG("First message on topic: %s", topic.c_str());
				auto topicPtr = std::make_shared<Topic>();
				topicPtr->name = topic;
				m_topics[topic] = topicPtr;
				message->topic = topicPtr;
			}
			else
				message->topic = it->second;
		}

		message->type = type;
		message->md5 = md5;
		message->flags = header.flags();
		message->payload.swap(data);
		message->counter = 0;

		m_callback(message);

		// Send ACK
		uint8_t ack = 1;
		if(write(fd, &ack, 1) != 1)
		{
			ROS_ERROR("Could not send ACK, ignoring (this is bad though!): %s", strerror(errno));
		}
	}
}

void TCPReceiver::setCallback(const Callback& cb)
{
	m_callback = cb;
}

}
