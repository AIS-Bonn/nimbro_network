// TCP sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_SENDER_H
#define TCP_SENDER_H

#include <ros/node_handle.h>
#include <topic_tools/shape_shifter.h>
#include <arpa/inet.h>

#include "tcp_packet.h"

namespace sb_topic_transport
{

class TCPSender
{
public:
	TCPSender();
	~TCPSender();

	bool connect();

	void send(const std::string& topic, const topic_tools::ShapeShifter& shifter);
private:
	ros::NodeHandle m_nh;
	int m_fd;
	sockaddr_in m_addr;
	std::vector<ros::Subscriber> m_subs;
};

}

#endif
