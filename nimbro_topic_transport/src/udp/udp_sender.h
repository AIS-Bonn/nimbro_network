// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <arpa/inet.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <ros/rate.h>
#include <ros/wall_timer.h>
#include <ros/publisher.h>

#include <deque>
#include <map>

#include <boost/thread.hpp>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

class TopicSender;

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	uint16_t allocateMessageID();
	bool send(const void* data, uint32_t size, const std::string& topic);

	inline bool duplicateFirstPacket() const
	{ return m_duplicateFirstPacket; }

	inline double fec() const
	{ return m_fec; }
private:
	void relay();
	bool internalSend(const void* data, uint32_t size, const std::string& topic);

	void updateStats();

	uint16_t m_msgID;
	int m_fd;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;
	ros::Time m_lastTime;
	bool m_duplicateFirstPacket;
	std::vector<TopicSender*> m_senders;

	bool m_relayMode;
	double m_relayRate;
	int m_relayTokensPerStep;
	uint64_t m_relayTokens;

	std::deque<std::vector<uint8_t>> m_relayBuffer;
	std::deque<std::string> m_relayNameBuffer;
	unsigned int m_relayIndex;

	boost::thread m_relayThread;
	bool m_relayThreadShouldExit;

	double m_fec;

	nimbro_topic_transport::SenderStats m_stats;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;
	uint64_t m_sentBytesInStatsInterval;

	// Count sent bytes for each topic seperately for logging
	std::map<std::string, uint64_t> m_sentTopicBytesInStatsInterval;
};

}

#endif
