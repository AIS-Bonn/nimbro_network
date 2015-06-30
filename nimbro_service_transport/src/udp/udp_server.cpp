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
		std::stringstream ss;
		ss << "Could not select(): " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	if(ret > 0)
	{
		handlePacket();
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

	uint8_t addrBuf[64];
	memset(addrBuf, 0, sizeof(addrBuf));
	socklen_t addr_len = sizeof(addrBuf);

	int ret = recvfrom(m_fd, m_buffer.data(), m_buffer.size(), 0, (sockaddr*)addrBuf, &addr_len);
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

	std::vector<uint8_t> cmp_buf(sizeof(ServiceCallResponse));
	ServiceCallResponse* cmp_response = reinterpret_cast<ServiceCallResponse*>(cmp_buf.data());

	cmp_response->timestamp = req->timestamp;
	cmp_response->counter = req->counter;

	auto it = std::lower_bound(m_responseList.begin(), m_responseList.end(), cmp_buf,
		[&](const std::vector<uint8_t>& a, const std::vector<uint8_t>& b)
		{
			const ServiceCallResponse& rA = *(reinterpret_cast<const ServiceCallResponse*>(a.data()));
			const ServiceCallResponse& rB = *(reinterpret_cast<const ServiceCallResponse*>(b.data()));

			if(rA.timestamp() < rB.timestamp())
				return true;
			else if(rA.timestamp() > rB.timestamp())
				return false;

			return rA.counter < rB.counter;
		}
	);

	const ServiceCallResponse* itResp = 0;
	if(it != m_responseList.end())
	{
		itResp = reinterpret_cast<const ServiceCallResponse*>(it->data());
	}

	if(!itResp || itResp->timestamp() != req->timestamp() || itResp->counter != req->counter)
	{
		std::string name(reinterpret_cast<char*>(m_buffer.data() + sizeof(ServiceCallRequest)), req->name_length());

		ros::ServiceClientOptions ops(name, "*", false, ros::M_string());
		ros::ServiceClient client = ros::NodeHandle().serviceClient(ops);

		boost::shared_array<uint8_t> array(new uint8_t[req->request_length()]);
		memcpy(array.get(), m_buffer.data() + sizeof(ServiceCallRequest) + req->name_length(), req->request_length());

		ros::SerializedMessage msg_request(array, req->request_length());
		ros::SerializedMessage msg_response;

		bool ok = client.call(msg_request, msg_response, std::string("*"));

		topic_tools::ShapeShifter response;
		ros::serialization::deserializeMessage(msg_response, response);

		ros::SerializedMessage msg_service_response;

		msg_service_response = ros::serialization::serializeServiceResponse(ok, response);

		std::vector<uint8_t> responseBuf(sizeof(ServiceCallResponse) + msg_service_response.num_bytes);

		ServiceCallResponse* resp = reinterpret_cast<ServiceCallResponse*>(responseBuf.data());
		resp->response_length = msg_service_response.num_bytes;
		resp->timestamp = req->timestamp;
		resp->counter = req->counter;

		memcpy(responseBuf.data() + sizeof(ServiceCallResponse), msg_service_response.buf.get(), msg_service_response.num_bytes);

		ret = sendto(m_fd, responseBuf.data(), responseBuf.size(), 0, (sockaddr*)addrBuf, addr_len);
		if(ret < 0)
		{
			ROS_ERROR("Could not send(): %s", strerror(errno));
			return;
		}

		m_responseList.push_back(std::move(responseBuf));
	}
	else
	{
		ret = sendto(m_fd, it->data(), it->size(), 0, (sockaddr*)addrBuf, addr_len);
		if(ret < 0)
		{
			ROS_ERROR("Could not send(): %s", strerror(errno));
			return;
		}
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
