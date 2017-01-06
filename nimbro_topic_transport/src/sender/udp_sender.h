// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_UDP_SENDER_H
#define TT_UDP_SENDER_H

#include <mutex>

#include <arpa/inet.h>

#include "message.h"

namespace nimbro_topic_transport
{

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	void send(const Message::ConstPtr& msg);
private:
	std::mutex m_mutex;

	uint16_t m_msgID = 0; //!< current message ID

	//! @name Socket stuff
	//@{
	int m_fd;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;
	//@}

	ros::Time m_lastTime;

	//! @name Statistics
	//@{
	void updateStats();

	nimbro_topic_transport::SenderStats m_stats;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;
	uint64_t m_sentBytesInStatsInterval;

	// Count sent bytes for each topic seperately for logging
	std::map<std::string, uint64_t> m_sentTopicBytesInStatsInterval;
	//@}

	//! @name FEC
	//@{
	double m_fec;
	//@}
};

}

#endif
