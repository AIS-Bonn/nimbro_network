// Receive UDP packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_RECEIVER_UDP_RECEIVER_H
#define TT_RECEIVER_UDP_RECEIVER_H

#include "../udp_packet.h"

#include <thread>
#include <functional>
#include <mutex>

#include <netinet/in.h>

#include <nimbro_topic_transport/DumpLog.h>

#include <ros/service_server.h>

namespace ros
{
class NodeHandle;
}

namespace nimbro_topic_transport
{

class UDPReceiver
{
public:
	typedef std::function<void(const Packet::Ptr&)> Callback;

	explicit UDPReceiver(ros::NodeHandle& nh);
	~UDPReceiver();

	UDPReceiver(const UDPReceiver&) = delete;
	UDPReceiver& operator=(const UDPReceiver&) = delete;

	void setCallback(const Callback& cb);

	void start();
private:
	bool dumpLog(DumpLogRequest& req, DumpLogResponse& resp);
	void thread();

	int m_fd;
	std::thread m_thread;
	bool m_shouldExit = false;

	Callback m_callback;

	struct LogEntry
	{
		std::uint16_t messageID = 0;
		std::uint32_t symbolID = 0;
		ros::Time receiptTime;
	};

	std::mutex m_logMutex;
	std::size_t m_logBufferCount = 0;
	std::size_t m_logBufferOffset = 0;
	std::vector<LogEntry> m_logBuffer;

	ros::ServiceServer m_srv_dumpLog;
};

}

#endif
