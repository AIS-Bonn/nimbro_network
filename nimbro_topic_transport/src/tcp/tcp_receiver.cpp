// TCP receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_receiver.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "tcp_packet.h"
#include "../topic_info.h"
#include <topic_tools/shape_shifter.h>

#include <bzlib.h>

#include <nimbro_topic_transport/CompressedMsg.h>

namespace nimbro_topic_transport
{

static bool sureRead(int fd, void* dest, ssize_t size)
{
	uint8_t* destWPtr = (uint8_t*)dest;
	while(size != 0)
	{
		ssize_t ret = read(fd, destWPtr, size);

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

		size -= ret;
		destWPtr += ret;
	}

	return true;
}


TCPReceiver::TCPReceiver()
 : m_nh("~")
 , m_receivedBytesInStatsInterval(0)
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

	m_nh.param("keep_compressed", m_keepCompressed, false);

	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = ros::this_node::getName();
	m_stats.protocol = "TCP";
	m_stats.host = hostnameBuf;
	m_stats.local_port = port;
	m_stats.fec = false;

	m_nh.param("label", m_stats.label, std::string());

	m_pub_stats = m_nh.advertise<ReceiverStats>("/network/receiver_stats", 1);
	m_statsInterval = ros::WallDuration(2.0);
	m_statsTimer = m_nh.createWallTimer(m_statsInterval,
		boost::bind(&TCPReceiver::updateStats, this)
	);

	m_nh.param("topic_prefix", m_topicPrefix, std::string());
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
				m_stats.remote = nameBuf;
				m_stats.remote_port = atoi(serviceBuf);
			}
			else
			{
				ROS_ERROR("Could not resolve remote address to name");
				m_stats.remote = "unknown";
				m_stats.remote_port = -1;
			}
			ros::WallTime endLookup = ros::WallTime::now();

			// Warn if lookup takes up time (otherwise the user does not know
			// what is going on)
			if(endLookup - startLookup > ros::WallDuration(1.0))
			{
				ROS_WARN("Reverse address lookup took more than a second. "
					"Consider adding '%s' to /etc/hosts",
					m_stats.remote.c_str()
				);
			}
		}

		ClientHandler* handler = new ClientHandler(client_fd, m_topicPrefix);
		handler->setKeepCompressed(m_keepCompressed);

		m_handlers.push_back(handler);
	}
}

TCPReceiver::ClientHandler::ClientHandler(int fd, const std::string& topicPrefix)
 : m_fd(fd)
 , m_uncompressBuf(1024)
 , m_running(true)
 , m_keepCompressed(false)
 , m_topicPrefix(topicPrefix)
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

		ROS_DEBUG("Got msg with flags: %d", header.flags());

		if(m_keepCompressed && (header.flags() & TCP_FLAG_COMPRESSED))
		{
			CompressedMsg compressed;
			compressed.type = type;
			memcpy(compressed.md5.data(), header.topic_md5sum, sizeof(header.topic_md5sum));
			compressed.data.swap(data);

			std::map<std::string, ros::Publisher>::iterator it = m_pub.find(topic);
			if(it == m_pub.end())
			{
				ros::NodeHandle nh;
				ros::Publisher pub = nh.advertise<CompressedMsg>(m_topicPrefix + topic, 2);
				m_pub[topic] = pub;

				pub.publish(compressed);
			}
			else
				it->second.publish(compressed);
		}
		else
		{
			topic_tools::ShapeShifter shifter;

			if(header.flags() & TCP_FLAG_COMPRESSED)
			{
				int ret = 0;
				unsigned int len = m_uncompressBuf.size();

				while(1)
				{
					ret = BZ2_bzBuffToBuffDecompress((char*)m_uncompressBuf.data(), &len, (char*)data.data(), data.size(), 0, 0);

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
					ROS_ERROR("Could not decompress bz2 data (reason %d), dropping msg", ret);
					continue;
				}

				ROS_DEBUG("decompress %d KiB (%d) to %d KiB (%d)", (int)data.size() / 1024, (int)data.size(), (int)len / 1024, (int)len);
				m_uncompressBuf.resize(len);

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
				ROS_DEBUG("Advertising new topic '%s'", (m_topicPrefix + topic).c_str());
				std::string msgDef = topic_info::getMsgDef(type);

				ros::NodeHandle nh;

				ros::AdvertiseOptions options(
					m_topicPrefix + topic,
					2,
					md5,
					type,
					topic_info::getMsgDef(type)
				);

				// It will take subscribers some time to connect to our publisher.
				// Therefore, latch messages so they will not be lost.
				// No, this is often unexpected. Instead, wait before publishing.
	// 			options.latch = true;

				m_pub[topic] = nh.advertise(options);
				it = m_pub.find(topic);

				sleep(1);
			}

			it->second.publish(shifter);
		}

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

void TCPReceiver::updateStats()
{
	m_stats.header.stamp = ros::Time::now();

	uint64_t totalBytes = 0;
	for(auto handler : m_handlers)
	{
		totalBytes += handler->bytesReceived();
		handler->resetByteCounter();
	}

	m_stats.bandwidth = totalBytes / m_statsInterval.toSec();

	m_stats.drop_rate = 0;

	// If there is no connection yet, drop the stats msg
	if(m_handlers.empty())
		return;

	m_pub_stats.publish(m_stats);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tcp_receiver");

	nimbro_topic_transport::TCPReceiver recv;

	recv.run();

	return 0;
}
