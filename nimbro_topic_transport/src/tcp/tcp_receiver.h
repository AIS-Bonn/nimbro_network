// TCP receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_RECEIVER_H
#define TCP_RECEIVER_H

#include <list>
#include <ros/node_handle.h>
#include <boost/thread.hpp>

#include <nimbro_topic_transport/ReceiverStats.h>

namespace nimbro_topic_transport
{

class TCPReceiver
{
public:
	TCPReceiver();
	~TCPReceiver();

	void run();
private:
	class ClientHandler
	{
	public:
		ClientHandler(int fd, const std::string& topicPrefix);
		~ClientHandler();
		void start();
		void run();

		void setKeepCompressed(bool keep)
		{ m_keepCompressed = keep; }

		bool isRunning() const;

		inline uint64_t bytesReceived() const
		{ return m_bytesReceived; }

		inline void resetByteCounter()
		{ m_bytesReceived = 0; }
	private:
		int m_fd;
		boost::thread m_thread;
		std::map<std::string, ros::Publisher> m_pub;
		std::vector<uint8_t> m_uncompressBuf;
		bool m_running;
		bool m_keepCompressed;
		uint64_t m_bytesReceived;
		std::string m_topicPrefix;
	};

	void updateStats();

	int m_fd;
	std::list<ClientHandler*> m_handlers;
	ros::NodeHandle m_nh;

	bool m_keepCompressed;

	nimbro_topic_transport::ReceiverStats m_stats;
	uint64_t m_receivedBytesInStatsInterval;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;

	std::string m_topicPrefix;
};

}

#endif
