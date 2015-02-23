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

#include <ros/console.h>
#include <stdio.h>
#include <errno.h>

#include <XmlRpcValue.h>

#include <signal.h>

namespace nimbro_topic_transport
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
	
	bool relay_mode;
	nh.param("relay_mode", relay_mode, false);

	std::string dest_host;
	nh.param("destination_addr", dest_host, std::string("192.168.178.255"));

	int dest_port;
	nh.param("destination_port", dest_port, 5050);

	if(nh.hasParam("source_port"))
	{
		int source_port;
		if(!nh.getParam("source_port", source_port))
		{
			ROS_FATAL("Invalid source_port");
			throw std::runtime_error("Invalid source port");
		}

		sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		addr.sin_port = htons(source_port);

		if(bind(m_fd, (const sockaddr*)&addr, sizeof(addr)) != 0)
		{
			ROS_FATAL("Could not bind to source port: %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}
	}

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

		int flags = 0;
		if(relay_mode)
		{
			flags |= UDP_FLAG_RELAY_MODE;
		}
		
		bool resend = false;

		double rate = 100.0;
		if(list[i].hasMember("rate"))
			rate = list[i]["rate"];

		if(list[i].hasMember("compress") && ((bool)list[i]["compress"]))
			flags |= UDP_FLAG_COMPRESSED;

		if(list[i].hasMember("resend") && ((bool)list[i]["resend"]))
			resend = true;

		m_senders.push_back(new TopicSender(this, &nh, list[i]["name"], rate, resend, flags));
	}

	nh.param("duplicate_first_packet", m_duplicateFirstPacket, false);
}

UDPSender::~UDPSender()
{
	for(unsigned int i = 0; i < m_senders.size(); ++i)
		delete m_senders[i];
}

uint16_t UDPSender::allocateMessageID()
{
	return m_msgID++;
}

bool UDPSender::send(void* data, uint32_t size)
{
	ros::Time now = ros::Time::now();
	ros::Duration delta = now - m_lastTime;

	if(delta < ros::Duration(0.008))
	{
		m_sleepCounter++;
		delta.sleep();

		if(m_sleepCounter > 125)
		{
			m_sleepCounter = 0;
			ROS_ERROR("UDPSender: the 8ms rate limit is limiting communication. Please send fewer data or increase the limit!");
		}
	}
	else
		m_sleepCounter = 0;

	if(sendto(m_fd, data, size, 0, (sockaddr*)&m_addr, sizeof(m_addr)) != size)
	{
		ROS_ERROR("Could not send data of size %d: %s", size, strerror(errno));
		return false;
	}

	return true;
}

uint32_t UDPSender::getAllTopicsLastDataSize()
{
	uint32_t size = 0;
	
	for(uint32_t i = 0; i < m_senders.size(); ++i)
	{
		size += m_senders[i]->getLastDataSize();
	}
	
	return size;
}

void UDPSender::sendAllTopicsLastData()
{
	for(uint32_t i = 0; i < m_senders.size(); ++i)
	{
		m_senders[i]->sendLastData();
	}
}

void interrupt_handler(int s)
{
	exit(0);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_sender");

	ros::NodeHandle nh("~");
	bool relay_mode;
	nh.param("relay_mode", relay_mode, false);

// 	signal(SIGINT, &nimbro_topic_transport::interrupt_handler);

	nimbro_topic_transport::UDPSender sender;
// 	nimbro_topic_transport::BandwidthControl bwc(10, 100,
// 		&nimbro_topic_transport::UDPSender::getAllTopicsLastDataSize,
// 		&nimbro_topic_transport::UDPSender::sendAllTopicsLastData, &sender);

// 	if(relay_mode)
// 	{
// 		while(1)
// 		{
// 			ros::spinOnce();
// 			bwc.send();
// 		}
// 	}
// 	else
	{
		ros::spin();
	}

	return 0;
}
