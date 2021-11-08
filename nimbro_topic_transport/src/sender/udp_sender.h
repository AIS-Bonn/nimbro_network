// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_UDP_SENDER_H
#define TT_UDP_SENDER_H

#include <mutex>

#include <arpa/inet.h>

#include "../message.h"
#include "../udp_packet.h"

namespace nimbro_topic_transport
{

class UDPSender
{
public:
	explicit UDPSender(ros::NodeHandle& nh);
	~UDPSender();

	void send(const std::vector<Packet::Ptr>& packets);
private:
	void printStats();

	//! @name Socket stuff
	//@{
	int m_fd;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;
	//@}

	std::mutex m_mutex;
	uint32_t m_packetID = 0;

	ros::SteadyTimer m_statTimer;
	uint64_t m_statPackets = 0;
	ros::Duration m_statDelay;
};

}

#endif
