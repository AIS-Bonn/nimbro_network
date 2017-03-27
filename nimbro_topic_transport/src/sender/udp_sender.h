// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_UDP_SENDER_H
#define TT_UDP_SENDER_H

#include <mutex>

#include <arpa/inet.h>

#include "topic.h"
#include "../message.h"
#include "../udp_packet.h"

namespace nimbro_topic_transport
{

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	void send(const std::vector<Packet::Ptr>& packets);
private:
	//! @name Socket stuff
	//@{
	int m_fd;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;
	//@}
};

}

#endif
