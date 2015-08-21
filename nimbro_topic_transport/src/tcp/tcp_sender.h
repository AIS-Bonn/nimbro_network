// TCP sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_SENDER_H
#define TCP_SENDER_H

#include <ros/node_handle.h>
#include <topic_tools/shape_shifter.h>
#include <arpa/inet.h>

#include "tcp_packet.h"

#include <config_server/parameter.h>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

class TCPSender
{
public:
	TCPSender();
	~TCPSender();

	bool connect();

	void send(const std::string& topic, int flags, const topic_tools::ShapeShifter& shifter);
private:
	void updateStats();

	ros::NodeHandle m_nh;
	int m_fd;

	int m_addrFamily;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;

	int m_sourcePort;
	std::vector<ros::Subscriber> m_subs;
	std::map<std::string, boost::shared_ptr<config_server::Parameter<bool>>> m_enableTopic;
	std::vector<uint8_t> m_packet;
	std::vector<uint8_t> m_compressionBuf;

	nimbro_topic_transport::SenderStats m_stats;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;
	uint64_t m_sentBytesInStatsInterval;
};

}

#endif
