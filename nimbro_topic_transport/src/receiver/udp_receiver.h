// Receive UDP packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_RECEIVER_UDP_RECEIVER_H
#define TT_RECEIVER_UDP_RECEIVER_H

#include "../udp_packet.h"

#include <thread>

#include <netinet/in.h>

namespace nimbro_topic_transport
{

class UDPReceiver
{
public:
	typedef std::function<void(const Packet::Ptr&)> Callback;

	UDPReceiver();
	~UDPReceiver();

	void setCallback(const Callback& cb);
private:
	void thread();

	int m_fd;
	std::thread m_thread;
	bool m_shouldExit = false;

	sockaddr_storage m_remoteAddr;
	socklen_t m_remoteAddrLen;

	Callback m_callback;
};

}

#endif
