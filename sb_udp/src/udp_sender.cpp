// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"
#include "topic_sender.h"

#include <ros/init.h>
#include <ros/node_handle.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>

#include <ros/console.h>
#include <stdio.h>
#include <errno.h>

#include <XmlRpcValue.h>

namespace sb_udp
{

UDPSender::UDPSender()
 : m_msgID(0)
{
	ros::NodeHandle nh("~");

	m_fd = socket(AF_INET, SOCK_DGRAM, 0);
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

	std::string dest_host;
	nh.param("destination_addr", dest_host, std::string("192.168.178.255"));

	int dest_port;
	nh.param("destination_port", dest_port, 5050);

	memset(&m_addr, 0, sizeof(m_addr));
	m_addr.sin_addr.s_addr = inet_addr(dest_host.c_str());
	m_addr.sin_port = htons(dest_port);
	m_addr.sin_family = AF_INET;


	XmlRpc::XmlRpcValue list;
	nh.getParam("topics", list);

	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int32_t i = 0; i < list.size(); ++i)
	{
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		ROS_ASSERT(list[i].hasMember("name"));

		double rate = 100.0;
		if(list[i].hasMember("rate"))
			rate = list[i]["rate"];

		new TopicSender(this, &nh, list[i]["name"], rate);
	}
}

UDPSender::~UDPSender()
{
}

uint16_t UDPSender::allocateMessageID()
{
	return m_msgID++;
}

bool UDPSender::send(void* data, uint32_t size)
{
	if(sendto(m_fd, data, size, 0, (sockaddr*)&m_addr, sizeof(m_addr)) != size)
	{
		ROS_ERROR("Could not send data of size %d: %s", size, strerror(errno));
		return false;
	}

	return true;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_sender");

	sb_udp::UDPSender sender;

	ros::spin();

	return 0;
}
