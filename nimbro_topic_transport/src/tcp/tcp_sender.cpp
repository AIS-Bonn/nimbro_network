// TCP sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_sender.h"
#include "../topic_info.h"

#include <bzlib.h>

#include <netinet/tcp.h>
#include <boost/algorithm/string/replace.hpp>

#include "ros/message_traits.h"

#include <netdb.h>

namespace nimbro_topic_transport
{

TCPSender::TCPSender()
 : m_nh("~")
 , m_fd(-1)
 , m_sentBytesInStatsInterval(0)
{
	std::string addr;
	if(!m_nh.getParam("destination_addr", addr) && !m_nh.getParam("address", addr))
	{
		ROS_FATAL("tcp_sender needs an 'destination_addr' parameter!");
		throw std::runtime_error("tcp_sender needs an 'destination_addr' parameter!");
	}

	int port;
	if(!m_nh.getParam("destination_port", port) && !m_nh.getParam("port", port))
	{
		ROS_FATAL("tcp_sender needs a 'destination_port' parameter!");
		throw std::runtime_error("tcp_sender needs a 'destination_port' parameter!");
	}

	std::string portStr = boost::lexical_cast<std::string>(port);

	if(m_nh.hasParam("source_port"))
	{
		if(!m_nh.getParam("source_port", m_sourcePort))
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

	XmlRpc::XmlRpcValue list;
	m_nh.getParam("topics", list);

	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int32_t i = 0; i < list.size(); ++i)
	{
		XmlRpc::XmlRpcValue& entry = list[i];

		ROS_ASSERT(entry.getType() == XmlRpc::XmlRpcValue::TypeStruct);
		ROS_ASSERT(entry.hasMember("name"));

		std::string topic = entry["name"];
		int flags = 0;

		if(entry.hasMember("compress") && ((bool)entry["compress"]) == true)
			flags |= TCP_FLAG_COMPRESSED;

		boost::function<void(const ros::MessageEvent<topic_tools::ShapeShifter const>&)> func;
		func = boost::bind(&TCPSender::messageCallback, this, topic, flags, _1);

		ros::SubscribeOptions options;
		options.initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(topic, 20, func);

		if(entry.hasMember("type"))
		{
			std::string type = entry["type"];

			std::string md5 = topic_info::getMd5Sum(type);
			if(md5.empty())
			{
				ROS_ERROR("Could not get md5 sum of topic type '%s'", type.c_str());
				continue;
			}

			options.datatype = type;
			options.md5sum = md5;
		}

		m_subs.push_back(m_nh.subscribe(options));

#if WITH_CONFIG_SERVER
		bool enabled = true;
		if (entry.hasMember("enable"))
			enabled = entry["enable"];

		std::string parameterName(topic);
		boost::replace_first(parameterName, "/", "");
		boost::replace_all(parameterName, "/", "_");

		boost::shared_ptr<config_server::Parameter<bool>> parameter( new config_server::Parameter<bool>(parameterName, enabled));
		m_enableTopic[topic] = parameter;
#endif
	}

	if (m_nh.hasParam("ignored_publishers")) {
		m_nh.getParam("ignored_publishers", m_ignoredPubs);
		for (std::vector<std::string>::iterator ignoredPublisher = m_ignoredPubs.begin();
			 	ignoredPublisher != m_ignoredPubs.end(); ignoredPublisher++) {
			*ignoredPublisher = ros::names::resolve(*ignoredPublisher);
		}
	}

	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = ros::this_node::getName();
	m_stats.protocol = "TCP";
	m_stats.host = hostnameBuf;
	m_stats.destination = addr;
	m_stats.destination_port = port;
	m_stats.source_port = m_sourcePort;
	m_stats.fec = false;

	m_nh.param("label", m_stats.label, std::string());

	m_pub_stats = m_nh.advertise<SenderStats>("/network/sender_stats", 1);

	for(auto& pair : m_topicSendBytesInStatsInteral)
		pair.second = 0;

	m_statsInterval = ros::WallDuration(2.0);
	m_statsTimer = m_nh.createWallTimer(m_statsInterval,
		boost::bind(&TCPSender::updateStats, this)
	);
	m_statsTimer.start();
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
		ROS_ERROR("Could not connect: %s", strerror(errno));
		return false;
	}

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

		m_stats.source_port = atoi(serviceBuf);
	}

#ifdef TCP_USER_TIMEOUT
	int timeout = 8000;
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

class PtrStream
{
public:
	PtrStream(uint8_t* ptr)
	 : m_ptr(ptr)
	{}

	inline uint8_t* advance(int)
	{ return m_ptr; }
private:
	uint8_t* m_ptr;
};

void TCPSender::messageCallback(const std::string& topic, int flags,
		const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
#if WITH_CONFIG_SERVER
	if (! (*m_enableTopic[topic])() )
		return;
#endif

	// check if the message wasn't published by an ignored publisher
	std::string const &messagePublisher = event.getConnectionHeader()["callerid"];
	if (std::find(m_ignoredPubs.begin(), m_ignoredPubs.end(), messagePublisher) != m_ignoredPubs.end())
		return;

	send(topic, flags, event.getMessage());
}


void TCPSender::send(const std::string& topic, int flags, const topic_tools::ShapeShifter::ConstPtr& shifter)
{
#if WITH_CONFIG_SERVER
	if (! (*m_enableTopic[topic])() )
		return;
#endif

	std::string type = shifter->getDataType();
	std::string md5 = shifter->getMD5Sum();
	uint32_t size = shifter->size();

	uint32_t maxDataSize = size;

	if(flags & TCP_FLAG_COMPRESSED)
		maxDataSize = size + size / 100 + 1200; // taken from bzip2 docs

	m_packet.resize(
		sizeof(TCPHeader) + topic.length() + type.length() + maxDataSize
	);

	TCPHeader* header = (TCPHeader*)m_packet.data();

	// Fill in topic & type
	uint8_t* wptr = m_packet.data() + sizeof(TCPHeader);

	memcpy(wptr, topic.c_str(), topic.length());
	wptr += topic.length();

	memcpy(wptr, type.c_str(), type.length());
	wptr += type.length();

	if(flags & TCP_FLAG_COMPRESSED)
	{
		unsigned int len = m_packet.size() - (wptr - m_packet.data());

		m_compressionBuf.resize(shifter->size());
		PtrStream stream(m_compressionBuf.data());
		shifter->write(stream);

		if(BZ2_bzBuffToBuffCompress((char*)wptr, &len, (char*)m_compressionBuf.data(), m_compressionBuf.size(), 7, 0, 30) == BZ_OK)
		{
			header->data_len = len;
			wptr += len;
			size = len;
		}
		else
		{
			ROS_ERROR("Could not compress with bzip2 library, sending uncompressed");
			flags &= ~TCP_FLAG_COMPRESSED;
			memcpy(wptr, m_compressionBuf.data(), m_compressionBuf.size());
			header->data_len = m_compressionBuf.size();
			wptr += m_compressionBuf.size();
		}
	}
	else
	{
		PtrStream stream(wptr);
		shifter->write(stream);
		header->data_len = size;
		wptr += size;
	}

	header->topic_len = topic.length();
	header->type_len = type.length();
	header->data_len = size;
	header->flags = flags;
	topic_info::packMD5(md5, header->topic_md5sum);

	// Resize to final size
	m_packet.resize(
		wptr - m_packet.data()
	);

	// Try to send the packet
	for(int tries = 0; tries < 10; ++tries)
	{
		if(m_fd == -1)
		{
			if(!connect())
			{
				ROS_WARN("Connection failed, trying again");
				continue;
			}
		}

		if(write(m_fd, m_packet.data(), m_packet.size()) != (int)m_packet.size())
		{
			ROS_WARN("Could not send data, trying again");
			close(m_fd);
			m_fd = -1;
			continue;
		}
		m_sentBytesInStatsInterval += m_packet.size();
		m_topicSendBytesInStatsInteral[topic] += m_packet.size();

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

	ROS_ERROR("Could not send TCP packet. Dropping message from topic %s!", topic.c_str());
}

void TCPSender::updateStats()
{
	m_stats.header.stamp = ros::Time::now();
	m_stats.bandwidth = 8 * m_sentBytesInStatsInterval / m_statsInterval.toSec();
	m_stats.topics.clear();
	for(auto& pair : m_topicSendBytesInStatsInteral)
	{
		nimbro_topic_transport::TopicBandwidth tp;
		tp.name = pair.first;
		tp.bandwidth = pair.second / m_statsInterval.toSec();
		pair.second = 0;
		m_stats.topics.push_back(tp);
	}

	m_pub_stats.publish(m_stats);
	m_sentBytesInStatsInterval = 0;
}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tcp_sender");

	nimbro_topic_transport::TCPSender sender;

	ros::spin();

	return 0;
}

