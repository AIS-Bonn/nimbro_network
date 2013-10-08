// Client side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "service_client.h"
#include "protocol.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#define DATA_DEBUG 0

#include <linux/version.h>

namespace service_transport
{


static int sureRead(int fd, void* dest, size_t size)
{
	size_t readBytes = 0;
	while(size != 0)
	{
		int ret = read(fd, dest, size);
		if(ret <= 0)
			return ret;

		size -= ret;
		dest = ((uint8_t*)dest) + ret;
		readBytes += ret;
	}

	return readBytes;
}

static void sureWrite(int fd, const void* src, size_t size)
{
	if(write(fd, src, size) != (int)size)
	{
		perror("Could not write()");
		throw std::runtime_error("Could not write()");
	}
}

class CallbackHelper : public ros::ServiceCallbackHelper
{
public:
	CallbackHelper(const std::string& name, int fd)
	 : m_name(name)
	 , m_fd(fd)
	{}

	virtual bool call(ros::ServiceCallbackHelperCallParams& params)
	{
		protocol::ServiceCallRequest req;
		req.name_length = m_name.length();
		req.request_length = params.request.num_bytes + 4;

#if DATA_DEBUG
		ROS_INFO("Sending service call request with %d bytes of data", params.request.num_bytes);
		for(int i = 0; i < params.request.num_bytes; ++i)
			ROS_INFO(" %d: 0x%02X (%c)", i, params.request.buf.get()[i], params.request.buf.get()[i]);
#endif

		sureWrite(m_fd, &req, sizeof(req));
		sureWrite(m_fd, m_name.c_str(), m_name.length());
		sureWrite(m_fd, &params.request.num_bytes, 4); // FIXME: Not portable
		sureWrite(m_fd, params.request.buf.get(), params.request.num_bytes);

		protocol::ServiceCallResponse resp;
		sureRead(m_fd, &resp, sizeof(resp));

		boost::shared_array<uint8_t> data(new uint8_t[resp.response_length()]);
		sureRead(m_fd, data.get(), resp.response_length());

		params.response = ros::SerializedMessage(data, resp.response_length());

		return true;
	}
private:
	std::string m_name;
	int m_fd;
};

ServiceClient::ServiceClient()
 : m_nh("~")
{
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	if(fd < 0)
	{
		perror("Could not create socket");
		throw std::runtime_error("socket error");
	}

	std::string server;
	if(!m_nh.getParam("server", server))
	{
		throw std::logic_error("service_client needs a server parameter (IP of the server)");
	}

	sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(server.c_str());
	addr.sin_port = htons(6050);

	protocol::ServiceCallRequest req;
	req.name_length = 0;
	req.request_length = 0;

#if SB_SERVICE_TRANSPORT_FASTOPEN && LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	if(sendto(fd, &req, sizeof(req), MSG_FASTOPEN, (const sockaddr*)&addr, sizeof(addr)) < 0)
	{
		perror("Could not connect to server");
		throw std::runtime_error("socket error");
	}
#else
	if(connect(fd, (const sockaddr*)&addr, sizeof(addr)) != 0)
	{
		perror("Could not connect to server");
		throw std::runtime_error("socket error");
	}
	sureWrite(fd, &req, sizeof(req));
#endif

	protocol::ServiceCallResponse resp;
	int ret = sureRead(fd, &resp, sizeof(resp));
	if(ret <= 0)
	{
		perror("Could not read");
		throw std::runtime_error("socket error");
	}

	ROS_DEBUG("reading %d bytes of service list data", resp.response_length());

	size_t to_read = resp.response_length();
	while(to_read > 0)
	{
		protocol::ServiceDescription desc;
		int ret = sureRead(fd, &desc, sizeof(desc));
		if(ret == 0)
			break;

		if(ret < 0)
		{
			perror("Could not read");
			throw std::runtime_error("socket error");
		}

		std::string name(desc.name_length(), ' ');
		sureRead(fd, (char*)name.data(), desc.name_length());

		std::string type(desc.type_length(), ' ');
		sureRead(fd, (char*)type.data(), desc.type_length());

		ros::AdvertiseServiceOptions ops;
		ops.callback_queue = 0;
		ops.datatype = type;

		for(int i = 0; i < 4; ++i)
		{
			char buf[10];
			snprintf(buf, sizeof(buf), "%08x", desc.md5[i]());
			ops.md5sum += buf;
		}

		ops.helper = boost::make_shared<CallbackHelper>(name, fd);
		ops.req_datatype = ops.datatype + "Request";
		ops.res_datatype = ops.datatype + "Response";
		ops.service = "/remote" + name;

		ROS_DEBUG("Advertising service '%s'", ops.service.c_str());

		ros::ServiceServer srv = m_nh.advertiseService(ops);
		m_servers.push_back(srv);

		to_read -= sizeof(desc) + desc.name_length() + desc.type_length();
	}

	ROS_INFO("Service client initialized.");
}

ServiceClient::~ServiceClient()
{
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_client");

	ROS_WARN("service_client: sb_service_transport is a proof-of-concept. It is NOT safe!");

	service_transport::ServiceClient client;

	ros::spin();
	return 0;
}
