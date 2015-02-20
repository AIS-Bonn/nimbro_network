// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <arpa/inet.h>
#include <ros/time.h>

#include "bandwidth_control.h"

namespace nimbro_topic_transport
{

class TopicSender;

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	uint16_t allocateMessageID();
	bool send(void* data, uint32_t size);
	
	uint32_t getAllTopicsLastDataSize();
	void sendAllTopicsLastData();

	inline bool duplicateFirstPacket() const
	{ return m_duplicateFirstPacket; }
private:
	uint16_t m_msgID;
	int m_fd;
	sockaddr_in m_addr;
	ros::Time m_lastTime;
	int m_sleepCounter;
	bool m_duplicateFirstPacket;
	std::vector<TopicSender*> m_senders;
};

}

#endif
