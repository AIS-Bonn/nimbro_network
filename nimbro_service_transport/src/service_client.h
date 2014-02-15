// Client side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVICE_CLIENT_H
#define SERVICE_CLIENT_H

#include <ros/node_handle.h>

#include <arpa/inet.h>

namespace service_transport
{

class ServiceClient
{
public:
	ServiceClient();
	~ServiceClient();

	bool call(const std::string& name, ros::ServiceCallbackHelperCallParams& params);
private:
	ros::NodeHandle m_nh;
	std::vector<ros::ServiceServer> m_servers;

	int m_fd;
	sockaddr_in m_addr;
};

}

#endif
