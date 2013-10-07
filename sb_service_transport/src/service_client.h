// Client side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVICE_CLIENT_H
#define SERVICE_CLIENT_H

#include <ros/node_handle.h>

namespace service_transport
{

class ServiceClient
{
public:
	ServiceClient();
	~ServiceClient();
private:
	ros::NodeHandle m_nh;
	std::vector<ros::ServiceServer> m_servers;
};

}

#endif
