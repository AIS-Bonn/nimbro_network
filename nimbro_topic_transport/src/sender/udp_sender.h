// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_UDP_SENDER_H
#define TT_UDP_SENDER_H

#include <mutex>

#include <arpa/inet.h>

#include <nimbro_topic_transport/SetDestinations.h>
#include <nimbro_topic_transport/DumpLog.h>

#include "../message.h"
#include "../udp_packet.h"

namespace nimbro_topic_transport
{

class UDPSender
{
public:
	explicit UDPSender(ros::NodeHandle& nh);
	~UDPSender();

	void send(const Message::ConstPtr& msg, const std::vector<Packet::Ptr>& packets);
private:
	void sendStats();
	bool handleSetDestinations(SetDestinationsRequest& req, SetDestinationsResponse& resp);
	bool dumpLog(DumpLogRequest& req, DumpLogResponse& resp);
	bool setupSockets(const std::vector<std::string>& destination_addrs);

	ros::NodeHandle m_nh;

	std::string m_label;

	//! @name Socket stuff
	//@{
	struct Socket
	{
		Socket() {}

		~Socket()
		{ if(fd >= 0) close(fd); }

		Socket(const Socket&) = delete;
		Socket& operator=(const Socket&) = delete;

		Socket(Socket&& other)
		{
			fd = other.fd;
			addr = other.addr;
			addrLen = other.addrLen;
			destination = other.destination;
			source_port = other.source_port;
			other.fd = -1;
		}

		int fd = -1;
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

	struct LogEntry
	{
		const Topic* topic = nullptr;
		std::uint16_t messageID = 0;
		ros::Time receiptTime;
		ros::Time startSend;
		ros::Time endSend;
	};

	std::mutex m_logMutex;
	std::size_t m_logBufferCount = 0;
	std::size_t m_logBufferOffset = 0;
	std::vector<LogEntry> m_logBuffer;

	ros::ServiceServer m_srv_dumpLog;
};

}

#endif
