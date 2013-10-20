// TCP receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_receiver.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include "tcp_packet.h"
#include "../topic_info.h"
#include <topic_tools/shape_shifter.h>

#include <bzlib.h>

namespace sb_topic_transport
{

static bool sureRead(int fd, void* dest, ssize_t size)
{
	ssize_t ret = read(fd, dest, size);

	if(ret < 0)
	{
		ROS_ERROR("Could not read(): %s", strerror(errno));
		return false;
	}

	if(ret == 0)
	{
		// Client has closed connection (ignore silently)
		return false;
	}

	if(ret != size)
	{
		ROS_ERROR("Invalid read size %d (expected %d)", (int)ret, (int)size);
		return false;
	}

	return true;
}


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

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) != 0)
	{
		ROS_FATAL("Could not enable SO_REUSEADDR: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	ROS_DEBUG("Binding to :%d", port);

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
 , m_uncompressBuf(1024)
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

		if(!sureRead(m_fd, &header, sizeof(header)))
			return;

		std::vector<char> buf(header.topic_len + 1);
		if(!sureRead(m_fd, buf.data(), header.topic_len))
			return;
		buf[buf.size()-1] = 0;

		std::string topic(buf.data());

		buf.resize(header.type_len+1);
		if(!sureRead(m_fd, buf.data(), header.type_len))
			return;
		buf[buf.size()-1] = 0;

		std::string type(buf.data());

		std::string md5;
		topic_info::unpackMD5(header.topic_md5sum, &md5);

		std::vector<uint8_t> data(header.data_len);
		if(!sureRead(m_fd, data.data(), header.data_len))
			return;

		topic_tools::ShapeShifter shifter;

		ROS_INFO("Got msg with flags: %d", header.flags());

		if(header.flags() & TCP_FLAG_COMPRESSED)
		{
			int ret = 0;
			unsigned int len = m_uncompressBuf.size();

			while(1)
			{
				int ret = BZ2_bzBuffToBuffDecompress((char*)m_uncompressBuf.data(), &len, (char*)data.data(), data.size(), 0, 0);

				if(ret == BZ_OUTBUFF_FULL)
				{
					len = 4 * m_uncompressBuf.size();
					ROS_INFO("Increasing buffer size to %d KiB", (int)len / 1024);
					m_uncompressBuf.resize(len);
					continue;
				}
				else
					break;
			}

			if(ret != BZ_OK)
			{
				ROS_ERROR("Could not decompress bz2 data, dropping msg");
				continue;
			}

			ROS_INFO("decompress %d KiB to %d KiB", (int)data.size() / 1024, (int)len / 1024);

			VectorStream stream(&m_uncompressBuf);
			shifter.read(stream);
		}
		else
		{
			VectorStream stream(&data);
			shifter.read(stream);
		}

		ROS_DEBUG("Got message from topic '%s' (type '%s', md5 '%s')", topic.c_str(), type.c_str(), md5.c_str());

		shifter.morph(md5, type, "", "");

		std::map<std::string, ros::Publisher>::iterator it = m_pub.find(topic);
		if(it == m_pub.end())
		{
			ROS_DEBUG("Advertising new topic '%s'", topic.c_str());
			std::string msgDef = topic_info::getMsgDef(type);

			ros::NodeHandle nh;

			ros::AdvertiseOptions options(
				topic,
				2,
				md5,
				type,
				topic_info::getMsgDef(type)
			);

			// It will take subscribers some time to connect to our publisher.
			// Therefore, latch messages so they will not be lost.
			options.latch = true;

			m_pub[topic] = nh.advertise(options);
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
