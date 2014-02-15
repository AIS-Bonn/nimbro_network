// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <arpa/inet.h>
#include <ros/time.h>

namespace nimbro_topic_transport
{

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	uint16_t allocateMessageID();
	bool send(void* data, uint32_t size);
private:
	uint16_t m_msgID;
	int m_fd;
	sockaddr_in m_addr;
	ros::Time m_lastTime;
	int m_sleepCounter;
};

}

#endif
