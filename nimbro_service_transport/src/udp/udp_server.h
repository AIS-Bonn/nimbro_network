// UDP service server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <ros/node_handle.h>

namespace nimbro_service_transport
{

class UDPServer
{
public:
	UDPServer();
	~UDPServer();

	void step();
private:
	void handlePacket();

	ros::NodeHandle m_nh;

	int m_fd;

	std::vector<uint8_t> m_buffer;

	std::list<std::vector<uint8_t>> m_responseList;
};

}

#endif
