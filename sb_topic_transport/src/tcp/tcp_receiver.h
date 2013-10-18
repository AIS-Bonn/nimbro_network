// TCP receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_RECEIVER_H
#define TCP_RECEIVER_H

#include <list>
#include <ros/node_handle.h>
#include <boost/thread.hpp>

namespace sb_topic_transport
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
		ClientHandler(int fd);
		void start();
		void run();

		bool isRunning() const;
	private:
		int m_fd;
		boost::thread m_thread;
		std::map<std::string, ros::Publisher> m_pub;
		bool m_running;
	};

	int m_fd;
	std::list<ClientHandler*> m_handlers;
	ros::NodeHandle m_nh;
};

}

#endif
