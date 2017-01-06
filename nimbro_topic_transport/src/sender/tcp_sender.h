// Send messages over TCP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_TCP_SENDER_H
#define TT_TCP_SENDER_H

#include "message.h"

#include <mutex>

#include <arpa/inet.h>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

class TCPSender
{
public:
	TCPSender();
	~TCPSender();

	//! @note may be called from multiple threads
	void send(const Message::ConstPtr& msg);
private:
	bool connect();
	void updateStats();

	ros::NodeHandle m_nh;
	int m_fd;

	std::mutex m_mutex;

	int m_addrFamily;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;

	int m_sourcePort;

	nimbro_topic_transport::SenderStats m_stats;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;
	uint64_t m_sentBytesInStatsInterval;
	std::map<std::string, uint64_t> m_topicSendBytesInStatsInteral;
};

}

#endif
