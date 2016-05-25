// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"
#include "topic_sender.h"
#include "udp_packet.h"

#include <ros/init.h>
#include <ros/node_handle.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <netdb.h>

#include <ros/console.h>
#include <stdio.h>
#include <errno.h>

#include <XmlRpcValue.h>

#include <signal.h>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

UDPSender::UDPSender()
 : m_msgID(0)
 , m_sentBytesInStatsInterval(0)
{
	ros::NodeHandle nh("~");

	nh.param("relay_mode", m_relayMode, false);

	std::string dest_host;
	nh.param("destination_addr", dest_host, std::string("localhost"));

	int dest_port;
	nh.param("destination_port", dest_port, 5050);

	std::string dest_port_str = boost::lexical_cast<std::string>(dest_port);

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
	if(nh.hasParam("source_port"))
	{
		if(!nh.getParam("source_port", source_port))
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

	nh.param("fec", m_fec, 0.0);

	XmlRpc::XmlRpcValue list;
	nh.getParam("topics", list);

	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int32_t i = 0; i < list.size(); ++i)
	{
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		ROS_ASSERT(list[i].hasMember("name"));

		int flags = 0;

		bool resend = false;

		double rate = 0.0; // unlimited
		if(list[i].hasMember("rate"))
			rate = list[i]["rate"];

		if(list[i].hasMember("compress") && ((bool)list[i]["compress"]))
			flags |= UDP_FLAG_COMPRESSED;

		if(list[i].hasMember("resend") && ((bool)list[i]["resend"]))
			resend = true;

		bool enabled = true;
		if(list[i].hasMember("enable") && (!(bool)list[i]["enable"]))
			enabled = false;

		std::string type;
		if(list[i].hasMember("type"))
			type = (std::string)(list[i]["type"]);

		TopicSender* sender = new TopicSender(this, &nh, list[i]["name"], rate, resend, flags, enabled, type);

		if(m_relayMode)
			sender->setDirectTransmissionEnabled(false);

		m_senders.push_back(sender);
	}

	nh.param("duplicate_first_packet", m_duplicateFirstPacket, false);

	if(m_relayMode)
	{
		double target_bitrate;
		if(!nh.getParam("relay_target_bitrate", target_bitrate))
		{
			throw std::runtime_error("relay mode needs relay_target_bitrate param");
		}

		double relay_control_rate;
		nh.param("relay_control_rate", relay_control_rate, 100.0);

		m_relayTokens = 0;
		m_relayIndex = 0;
		m_relayTokensPerStep = target_bitrate / 8.0 / relay_control_rate;

		m_relayThreadShouldExit = false;
		m_relayRate = relay_control_rate;
		m_relayThread = boost::thread(boost::bind(&UDPSender::relay, this));

		ROS_INFO("udp_sender: relay mode configured with control rate %f, target bitrate %f bit/s and token increment %d",
			relay_control_rate, target_bitrate, m_relayTokensPerStep
		);
	}

	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = ros::this_node::getName();
	m_stats.protocol = "UDP";
	m_stats.host = hostnameBuf;
	m_stats.destination = dest_host;
	m_stats.destination_port = dest_port;
	m_stats.source_port = source_port;
	m_stats.fec = m_fec != 0.0;

	nh.param("label", m_stats.label, std::string());

	m_pub_stats = nh.advertise<nimbro_topic_transport::SenderStats>("/network/sender_stats", 1);

	m_statsInterval = ros::WallDuration(2.0);
	m_statsTimer = nh.createWallTimer(m_statsInterval,
		boost::bind(&UDPSender::updateStats, this)
	);
	m_statsTimer.start();
}

UDPSender::~UDPSender()
{
	if(m_relayMode)
	{
		m_relayThreadShouldExit = true;
		m_relayThread.join();
	}

	for(unsigned int i = 0; i < m_senders.size(); ++i)
		delete m_senders[i];
}

uint16_t UDPSender::allocateMessageID()
{
	return m_msgID++;
}

bool UDPSender::send(const void* data, uint32_t size, const std::string& topic)
{
	if(m_relayMode)
	{
		std::vector<uint8_t> packet(size);
		memcpy(packet.data(), data, size);

		m_relayBuffer.emplace_back(std::move(packet));
		m_relayNameBuffer.push_back(topic);
		return true;
	}
	else
	{
		return internalSend(data, size, topic);
	}
}

bool UDPSender::internalSend(const void* data, uint32_t size, const std::string& topic)
{
	if(sendto(m_fd, data, size, 0, (sockaddr*)&m_addr, m_addrLen) != size)
	{
		ROS_ERROR("Could not send data of size %d: %s", size, strerror(errno));
		return false;
	}

	m_sentBytesInStatsInterval += size;
	if (!topic.empty())
		m_sentTopicBytesInStatsInterval[topic] += size;

	return true;
}

void UDPSender::relay()
{
	ros::WallRate rate(m_relayRate);

	ROS_INFO("Relay thread starting...");

	while(!m_relayThreadShouldExit)
	{
		// New tokens! Bound to 100*m_relayTokensPerStep to prevent token buildup.
		m_relayTokens = std::min<uint64_t>(
			100*m_relayTokensPerStep,
			m_relayTokens + m_relayTokensPerStep
		);

		if(m_senders.empty())
			throw std::runtime_error("No senders configured");

		// While we have enough token, send something!
		while(1)
		{
			unsigned int tries = 0;
			bool noData = false;

			// Find a topic sender that has some data for us. Note that we
			// do not consume the message!
			while(m_relayBuffer.empty())
			{
				if(tries++ == m_senders.size())
				{
					noData = true;
					break; // No data yet
				}

				m_senders[m_relayIndex]->sendCurrentMessage();
				m_relayIndex = (m_relayIndex + 1) % m_senders.size();
			}

			if(noData)
				break;

			const std::vector<uint8_t>& packet = m_relayBuffer.front();

			// Get the topic name the packet is from
			const std::string& topic = m_relayNameBuffer.front();
			std::size_t sizeOnWire = packet.size() + 20 + 8;

			// out of tokens? Wait for next iteration.
			if(sizeOnWire > m_relayTokens)
				break;

			if(!internalSend(packet.data(), packet.size(), topic))
			{
				ROS_ERROR("Could not send packet");
				break;
			}

			// Consume tokens
			m_relayTokens -= sizeOnWire;
			m_relayBuffer.pop_front();
			m_relayNameBuffer.pop_front();
		}

		rate.sleep();
	}

	ROS_INFO("Relay thread exiting...");
}

void UDPSender::updateStats()
{
	m_stats.header.stamp = ros::Time::now();
	m_stats.bandwidth = 8 * m_sentBytesInStatsInterval / m_statsInterval.toSec();

	// Get Bytes per topic in the message
	m_stats.topics.clear();
	for(auto& pair : m_sentTopicBytesInStatsInterval)
	{
		nimbro_topic_transport::TopicBandwidth tp;
		tp.name = pair.first;
		tp.bandwidth = 8 * pair.second / m_statsInterval.toSec();
		pair.second = 0;
        m_stats.topics.emplace_back(tp);
	}

	m_pub_stats.publish(m_stats);
	m_sentBytesInStatsInterval = 0;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_sender");

	ros::NodeHandle nh("~");

	nimbro_topic_transport::UDPSender sender;

	ros::spin();

	return 0;
}

