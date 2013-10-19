// TCP receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_receiver.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include "tcp_packet.h"
#include "../topic_info.h"
#include <topic_tools/shape_shifter.h>

namespace sb_topic_transport
{

TCPReceiver::TCPReceiver()
 : m_nh("~")
{
	m_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(m_fd < 0)
	{
		ROS_FATAL("Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int port;
	m_nh.param("port", port, 5050);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	ROS_INFO("Binding to :%d", port);

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
}

void TCPReceiver::run()
{
	fd_set fds;

	while(ros::ok())
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

		int client_fd = accept(m_fd, 0, 0);
		m_handlers.push_back(
			new ClientHandler(client_fd)
		);
	}
}

TCPReceiver::ClientHandler::ClientHandler(int fd)
 : m_fd(fd)
 , m_running(true)
{
	m_thread = boost::thread(boost::bind(&ClientHandler::start, this));
}

TCPReceiver::ClientHandler::~ClientHandler()
{
	close(m_fd);
}

class VectorStream
{
public:
	VectorStream(std::vector<uint8_t>* vector)
	 : m_vector(vector)
	{}

	inline const uint8_t* getData()
	{ return m_vector->data(); }

	inline size_t getLength()
	{ return m_vector->size(); }
private:
	std::vector<uint8_t>* m_vector;
};

void TCPReceiver::ClientHandler::start()
{
	run();
	m_running = false;
}

void TCPReceiver::ClientHandler::run()
{
	while(1)
	{
		TCPHeader header;

		if(read(m_fd, &header, sizeof(header)) != sizeof(header))
		{
			ROS_ERROR("Could not read(): %s", strerror(errno));
			return;
		}

		std::vector<char> buf(header.topic_len + 1);
		if(read(m_fd, buf.data(), header.topic_len) != header.topic_len)
		{
			ROS_ERROR("Could not read(): %s", strerror(errno));
			return;
		}
		buf[buf.size()-1] = 0;

		std::string topic(buf.data());

		buf.resize(header.type_len+1);
		if(read(m_fd, buf.data(), header.type_len) != header.type_len)
		{
			ROS_ERROR("Could not read(): %s", strerror(errno));
			return;
		}
		buf[buf.size()-1] = 0;

		std::string type(buf.data());

		std::string md5;
		topic_info::unpackMD5(header.topic_md5sum, &md5);

		std::vector<uint8_t> data(header.data_len);
		if(read(m_fd, data.data(), header.data_len) != header.data_len)
		{
			ROS_ERROR("Could not read(): %s", strerror(errno));
			return;
		}

		topic_tools::ShapeShifter shifter;
		VectorStream stream(&data);
		shifter.read(stream);

		ROS_INFO("Got message from topic '%s' (type '%s', md5 '%s')", topic.c_str(), type.c_str(), md5.c_str());

		std::map<std::string, ros::Publisher>::iterator it = m_pub.find(topic);
		if(it == m_pub.end())
		{
			ROS_INFO("Advertising new topic '%s'", topic.c_str());

			ros::NodeHandle nh;
			ros::Publisher pub = shifter.advertise(nh, topic, 1);

			m_pub[topic] = pub;
			it = m_pub.find(topic);
		}

		it->second.publish(shifter);

		uint8_t ack = 1;
		if(write(m_fd, &ack, 1) != 1)
		{
			ROS_ERROR("Could not write(): %s", strerror(errno));
			return;
		}
	}
}

bool TCPReceiver::ClientHandler::isRunning() const
{
	return m_running;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tcp_receiver");

	sb_topic_transport::TCPReceiver recv;

	recv.run();

	return 0;
}
