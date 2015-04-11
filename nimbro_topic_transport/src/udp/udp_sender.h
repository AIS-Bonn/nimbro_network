// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <arpa/inet.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <ros/rate.h>

#include <deque>

#include <boost/thread.hpp>

namespace nimbro_topic_transport
{

class TopicSender;

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	uint16_t allocateMessageID();
	bool send(const void* data, uint32_t size);

	inline bool duplicateFirstPacket() const
	{ return m_duplicateFirstPacket; }
private:
	void relay();
	bool internalSend(const void* data, uint32_t size);

	uint16_t m_msgID;
	int m_fd;
	sockaddr_in m_addr;
	ros::Time m_lastTime;
	int m_sleepCounter;
	bool m_duplicateFirstPacket;
	std::vector<TopicSender*> m_senders;

	bool m_relayMode;
	double m_relayRate;
	int m_relayTokensPerStep;
	uint64_t m_relayTokens;

	std::deque<std::vector<uint8_t>> m_relayBuffer;
	unsigned int m_relayIndex;

	boost::thread m_relayThread;
	bool m_relayThreadShouldExit;
};

}

#endif
