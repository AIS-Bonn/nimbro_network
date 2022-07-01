// Send messages over TCP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_TCP_SENDER_H
#define TT_TCP_SENDER_H

#include "../message.h"

#include <mutex>

#include <arpa/inet.h>

namespace nimbro_topic_transport
{

class TCPSender
{
public:
	explicit TCPSender(ros::NodeHandle& nh);
	~TCPSender();

	//! @note may be called from multiple threads
	void send(const Message::ConstPtr& msg);
private:
	enum Verbosity
	{
		Print,
		Quiet
	};

	bool connect(Verbosity verbosity = Verbosity::Print);
	void updateStats();

	ros::NodeHandle m_nh;
	int m_fd;

	std::mutex m_mutex;

	int m_addrFamily;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;

	int m_sourcePort;
};

}

#endif
