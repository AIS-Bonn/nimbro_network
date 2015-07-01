// Server side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVICE_SERVER_H
#define SERVICE_SERVER_H

#include <boost/thread.hpp>
#include <ros/connection.h>

#include "client_handler.h"

namespace nimbro_service_transport
{

class ServiceServer
{
public:
	ServiceServer();
	~ServiceServer();

	void step();
private:
	int m_fd;
	std::vector<boost::shared_ptr<ClientHandler> > m_handlers;
};

}

#endif
