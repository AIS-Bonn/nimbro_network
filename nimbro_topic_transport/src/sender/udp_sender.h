// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_UDP_SENDER_H
#define TT_UDP_SENDER_H

#include <mutex>

#include <arpa/inet.h>

#include <nimbro_topic_transport/SetDestinations.h>

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
	void sendStats();
	bool handleSetDestinations(SetDestinationsRequest& req, SetDestinationsResponse& resp);
	bool setupSockets(const std::vector<std::string>& destination_addrs);

	ros::NodeHandle m_nh;

	std::string m_label;

	//! @name Socket stuff
	//@{
	struct Socket
	{
		~Socket()
		{ close(fd); }

		int fd;
		sockaddr_storage addr;
		socklen_t addrLen;

		std::string destination;

		int source_port = 0;
	};
	std::vector<Socket> m_sockets;

	std::string m_hostname;
	int m_destinationPort = 0;
	//@}

	std::mutex m_mutex;
	uint32_t m_packetID = 0;

	ros::SteadyTimer m_statTimer;
	ros::Time m_lastStatTime;
	uint64_t m_statPackets = 0;
	ros::Duration m_statDelay;

	std::map<std::string, std::uint64_t> m_topicBandwidth;

	ros::Publisher m_pub_stats;

	ros::ServiceServer m_srv_setDestinations;
};

}

#endif
