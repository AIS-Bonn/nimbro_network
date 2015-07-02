// UDP service server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_server.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <netinet/in.h>
#include <netinet/udp.h>

#include <arpa/inet.h>

#include <stdexcept>

#include <topic_tools/shape_shifter.h>

#include "protocol.h"

namespace nimbro_service_transport
{

UDPServer::UDPServer()
 : m_nh("~")
 , m_buffer(1024)
{
	int port;
	m_nh.param("port", port, 5000);

	m_fd = socket(AF_INET6, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		std::stringstream ss;
		ss << "Could not create socket: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	sockaddr_in6 addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin6_family = AF_INET6;
	addr.sin6_addr = IN6ADDR_ANY_INIT;
	addr.sin6_port = htons(port);

	if(bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		close(m_fd);
		std::stringstream ss;
		ss << "Could not bind socket: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

UDPServer::~UDPServer()
{
	close(m_fd);
}

void UDPServer::step()
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(m_fd, &fds);

	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 500 * 1000;

	int ret = select(m_fd + 1, &fds, 0, 0, &timeout);
	if(ret < 0)
	{
		// Silently ignore EINTR, EAGAIN
		if(errno == EINTR || errno == EAGAIN)
			return;

		std::stringstream ss;
		ss << "Could not select(): " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	if(ret > 0)
	{
		handlePacket();
	}

	// Cleanup old requestHandlers
	ros::Time now = ros::Time::now();

	auto it = m_requestList.begin();
	while(it != m_requestList.end())
	{
		auto& reqHandler = *it;

		if(now - reqHandler->receptionTime < ros::Duration(20.0))
			break;

		{
			boost::unique_lock<boost::mutex> lock(reqHandler->mutex);

			if(reqHandler->calling)
			{
				// This one is still active, keep it alive
				it++;
				continue;
			}

			reqHandler->serviceThread.join();
		}

		it = m_requestList.erase(it);
	}
}

void UDPServer::handlePacket()
{
	int packetSize;

	while(1)
	{
		if(ioctl(m_fd, FIONREAD, &packetSize) != 0)
		{
			if(errno == EAGAIN || errno == EINTR)
				continue;
			else
			{
				perror("FIONREAD");
				break;
			}
		}

		m_buffer.resize(packetSize);
		break;
	}

	sockaddr_storage addr;
	memset(&addr, 0, sizeof(addr));
	socklen_t addr_len = sizeof(addr);

	int ret = recvfrom(m_fd, m_buffer.data(), m_buffer.size(), 0, (sockaddr*)&addr, &addr_len);
	if(ret < 0)
	{
		ROS_ERROR("Could not recvfrom(): %s", strerror(errno));
		return;
	}

	m_buffer.resize(ret);

	if(m_buffer.size() < sizeof(ServiceCallRequest))
	{
		ROS_ERROR("Received short packet of size %lu", m_buffer.size());
		return;
	}

	const ServiceCallRequest* req = reinterpret_cast<ServiceCallRequest*>(m_buffer.data());

	if(m_buffer.size() < sizeof(ServiceCallRequest) + req->name_length)
	{
		ROS_ERROR("request header references name which is out-of-buffer");
		return;
	}

	if(m_buffer.size() < sizeof(ServiceCallRequest) + req->name_length() + req->request_length())
	{
		ROS_ERROR("request header references data which is out-of-buffer");
		return;
	}

	auto cmp = boost::make_shared<RequestHandler>(req->timestamp(), req->counter, "", ros::SerializedMessage());

	auto it = std::lower_bound(m_requestList.begin(), m_requestList.end(), cmp,
		[&](const boost::shared_ptr<RequestHandler>& a, const boost::shared_ptr<RequestHandler>& b)
		{
			if(a->timestamp < b->timestamp)
				return true;
			else if(a->timestamp > b->timestamp)
				return false;

			return a->counter < b->counter;
		}
	);

	boost::shared_ptr<RequestHandler> reqHandler;
	if(it != m_requestList.end())
	{
		reqHandler = *it;
	}

	if(!reqHandler || reqHandler->timestamp != req->timestamp() || reqHandler->counter != req->counter)
	{
		std::string name(reinterpret_cast<char*>(m_buffer.data() + sizeof(ServiceCallRequest)), req->name_length());

		boost::shared_array<uint8_t> array(new uint8_t[req->request_length()]);
		memcpy(array.get(), m_buffer.data() + sizeof(ServiceCallRequest) + req->name_length(), req->request_length());

		ros::SerializedMessage msg_request(array, req->request_length());

		reqHandler = boost::make_shared<RequestHandler>(req->timestamp(), req->counter, name, msg_request);
		reqHandler->fd = m_fd;
		memcpy(&reqHandler->addr, &addr, sizeof(addr));
		reqHandler->addrLen = addr_len;

		reqHandler->calling = true;
		reqHandler->serviceThread = boost::thread(boost::bind(&RequestHandler::call, reqHandler));

		reqHandler->receptionTime = ros::Time::now();

		m_requestList.push_back(reqHandler);
	}
	else
	{
		boost::unique_lock<boost::mutex> lock(reqHandler->mutex);

		if(reqHandler->calling)
			ROS_WARN("Received additional request for in-progress service call");
		else
		{
			ret = sendto(m_fd, reqHandler->response.data(), reqHandler->response.size(), 0, (sockaddr*)&addr, addr_len);
			if(ret < 0)
			{
				ROS_ERROR("Could not send(): %s", strerror(errno));
				return;
			}
		}
	}
}

void UDPServer::RequestHandler::call()
{
	ros::ServiceClientOptions ops(service, "*", false, ros::M_string());
	ros::ServiceClient client = ros::NodeHandle().serviceClient(ops);

	ros::SerializedMessage msg_response;

	bool ok = client.call(request, msg_response, std::string("*"));

	topic_tools::ShapeShifter deserializedResponse;
	ros::serialization::deserializeMessage(msg_response, deserializedResponse);

	ros::SerializedMessage msg_service_response;

	msg_service_response = ros::serialization::serializeServiceResponse(ok, deserializedResponse);

	boost::unique_lock<boost::mutex> lock(mutex);

	response.resize(sizeof(ServiceCallResponse) + msg_service_response.num_bytes);

	ServiceCallResponse* resp = reinterpret_cast<ServiceCallResponse*>(response.data());
	resp->response_length = msg_service_response.num_bytes;
	resp->timestamp = timestamp;
	resp->counter = counter;

	memcpy(response.data() + sizeof(ServiceCallResponse), msg_service_response.buf.get(), msg_service_response.num_bytes);

	calling = false;

	int ret = sendto(fd, response.data(), response.size(), 0, (sockaddr*)&addr, addrLen);
	if(ret < 0)
	{
		ROS_ERROR("Could not send(): %s", strerror(errno));
		return;
	}
}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_server");

	nimbro_service_transport::UDPServer server;

	while(ros::ok())
	{
		server.step();
		ros::spinOnce();
	}

	return 0;
}
