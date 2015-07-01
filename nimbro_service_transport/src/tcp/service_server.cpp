// Server side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "service_server.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <linux/version.h>

#include <ros/master.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service_manager.h>
#include <ros/transport/transport_tcp.h>
#include <ros/poll_manager.h>

#include <XmlRpcValue.h>

#include "protocol.h"

namespace nimbro_service_transport
{

ServiceServer::ServiceServer()
{
	ros::NodeHandle nh("~");

	m_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(m_fd < 0)
	{
		perror("Could not create socket");
		throw std::runtime_error("socket error");
	}

	int port;
	nh.param("port", port, 6050);

	sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) != 0)
	{
		perror("Could not enable SO_REUSEADDR");
		throw std::runtime_error("socket error");
	}

	while(bind(m_fd, (const sockaddr*)&addr, sizeof(addr)) != 0)
	{
		perror("Could not bind to port 6050, trying again");
		sleep(2);
	}

#if NIMBRO_SERVICE_TRANSPORT_FASTOPEN && LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	int qlen = 10;
	if(setsockopt(m_fd, SOL_TCP, TCP_FASTOPEN, &qlen, sizeof(qlen)) != 0)
	{
		perror("Could not enable TCP fast open");
		throw std::runtime_error("socket error");
	}
#endif

	if(listen(m_fd, 10) != 0)
	{
		perror("Could not listen()");
		throw std::runtime_error("socket error");
	}
}

ServiceServer::~ServiceServer()
{
	close(m_fd);
}

void ServiceServer::step()
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(m_fd, &fds);

	timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	int ret = select(m_fd+1, &fds, 0, 0, &timeout);

	if(ret < 0)
	{
		if(errno == EINTR || errno == EAGAIN)
			return;

		perror("Could not select()");
		throw std::runtime_error("select() error");
	}

	if(ret == 0)
		return;

	int client = accept(m_fd, 0, 0);

	m_handlers.push_back(
		boost::make_shared<ClientHandler>(client)
	);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_server");

	ros::NodeHandle nh("~");

	nimbro_service_transport::ServiceServer srv;

	while(ros::ok())
	{
		ros::spinOnce();
		srv.step();
	}

	return 0;
}
