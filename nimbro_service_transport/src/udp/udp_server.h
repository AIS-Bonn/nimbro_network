// UDP service server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <ros/node_handle.h>

#include <boost/thread.hpp>

#include <sys/socket.h>

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

	struct RequestHandler
	{
		RequestHandler(uint64_t timestamp, uint8_t counter, const std::string& service, const ros::SerializedMessage& request)
		 : timestamp(timestamp)
		 , counter(counter)
		 , service(service)
		 , request(request)
		 , calling(false)
		{}

		void call();

		bool operator<(const RequestHandler& other) const;

		int fd;
		sockaddr_storage addr;
		socklen_t addrLen;

		uint64_t timestamp;
		uint8_t counter;
		std::string service;
		ros::SerializedMessage request;

		bool calling;
		boost::thread serviceThread;

		boost::mutex mutex;
		std::vector<uint8_t> response;

		ros::Time receptionTime;
	};

	std::list<boost::shared_ptr<RequestHandler>> m_requestList;
};

}

#endif
