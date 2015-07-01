// Handles one client in a dedicated thread
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CLIENT_HANDLER_H
#define CLIENT_HANDLER_H

#include <boost/thread.hpp>
#include <ros/header.h>
#include <ros/connection.h>

namespace nimbro_service_transport
{

class ClientHandler
{
public:
	ClientHandler(int fd);
	~ClientHandler();
private:
	void run();
	void sendAvailableServices();

	bool handleHeader(const ros::ConnectionPtr& connection, const ros::Header& header);
	void handleHeaderWritten();

	int m_fd;
	bool m_gotHeader;
	ros::Header m_headerBuf;
	boost::thread m_thread;
};

}

#endif
