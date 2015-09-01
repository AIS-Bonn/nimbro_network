// Client side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVICE_CLIENT_H
#define SERVICE_CLIENT_H

#include <ros/node_handle.h>

#include <arpa/inet.h>

namespace nimbro_service_transport
{

class ServiceClient
{
public:
	ServiceClient();
	~ServiceClient();

	bool call(const std::string& name, ros::ServiceCallbackHelperCallParams& params);
private:
	void publishStatus(const std::string& service, uint8_t status);

	ros::NodeHandle m_nh;
	std::vector<ros::ServiceServer> m_servers;

	ros::Publisher m_pub_status;
	uint32_t m_currentCallID;

	int m_fd;

	sockaddr_storage m_addr;
	socklen_t m_addrLen;

	std::string m_host;
	std::string m_remote;
	int m_remotePort;
};

}

#endif
