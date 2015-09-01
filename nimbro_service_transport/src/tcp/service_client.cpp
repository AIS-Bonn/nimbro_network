// Client side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "service_client.h"
#include "protocol.h"
#include "../common.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>

#define DATA_DEBUG 0

#include <linux/version.h>

#include <ros/names.h>
#include <ros/package.h>

#include <nimbro_service_transport/ServiceStatus.h>

namespace nimbro_service_transport
{

class IOException : public std::runtime_error
{
public:
	explicit IOException()
	 : std::runtime_error("IO exception")
	{}
};


static int sureRead(int fd, void* dest, size_t size)
{
	size_t readBytes = 0;
	while(size != 0)
	{
		int ret = read(fd, dest, size);
		if(ret <= 0)
		{
			perror("Could not read()");
			throw IOException();
		}

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
		throw IOException();
	}
}

class CallbackHelper : public ros::ServiceCallbackHelper
{
public:
	CallbackHelper(const std::string& name, ServiceClient* client)
	 : m_name(name)
	 , m_client(client)
	{}

	virtual bool call(ros::ServiceCallbackHelperCallParams& params)
	{
		return m_client->call(m_name, params);
	}
private:
	std::string m_name;
	ServiceClient* m_client;
};

ServiceClient::ServiceClient()
 : m_nh("~")
 , m_currentCallID(0)
 , m_fd(-1)
{
	if(!m_nh.getParam("server", m_remote))
	{
		throw std::logic_error("service_client needs a server parameter (IP of the server)");
	}

	m_nh.param("port", m_remotePort, 6050);

	std::string portString = boost::lexical_cast<std::string>(m_remotePort);

	// Get local host name for visualization messages
	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_host = hostnameBuf;

	// Resolve remote address
	addrinfo hint;
	memset(&hint, 0, sizeof(hint));

	hint.ai_socktype = SOCK_STREAM;

	addrinfo* info = 0;
	if(getaddrinfo(m_remote.c_str(), portString.c_str(), &hint, &info) != 0 || !info)
	{
		std::stringstream ss;
		ss << "Could not resolve server: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	if(info->ai_addrlen > sizeof(m_addr))
		throw std::runtime_error("Invalid address length");

	memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
	m_addrLen = info->ai_addrlen;

	XmlRpc::XmlRpcValue list;
	m_nh.getParam("services", list);

	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int32_t i = 0; i < list.size(); ++i)
	{
		XmlRpc::XmlRpcValue& entry = list[i];
		ROS_ASSERT(entry.getType() == XmlRpc::XmlRpcValue::TypeStruct);
		ROS_ASSERT(entry.hasMember("name"));
		ROS_ASSERT(entry.hasMember("type"));

		std::string name = (std::string)entry["name"];
		std::string type = (std::string)entry["type"];

		ros::AdvertiseServiceOptions ops;
		ops.callback_queue = 0;
		ops.datatype = type;
		ops.md5sum = getServiceMD5(ops.datatype);
		ops.helper = boost::make_shared<CallbackHelper>(name, this);
		ops.req_datatype = ops.datatype + "Request";
		ops.res_datatype = ops.datatype + "Response";
		ops.service = name;

		ROS_DEBUG("Advertising service '%s'", ops.service.c_str());

		ros::ServiceServer srv = m_nh.advertiseService(ops);
		m_servers.push_back(srv);
	}

	m_pub_status = m_nh.advertise<ServiceStatus>("/network/service_status", 10);

	ROS_INFO("Service client initialized.");
}

ServiceClient::~ServiceClient()
{
}

bool ServiceClient::call(const std::string& name, ros::ServiceCallbackHelperCallParams& params)
{
	m_currentCallID++;

	protocol::ServiceCallRequest req;
	req.name_length = name.length();
	req.request_length = params.request.num_bytes + 4;

#if DATA_DEBUG
	ROS_INFO("Sending service call request with %d bytes of data", params.request.num_bytes);
	for(int i = 0; i < params.request.num_bytes; ++i)
		ROS_INFO(" %d: 0x%02X (%c)", i, params.request.buf.get()[i], params.request.buf.get()[i]);
#endif

	publishStatus(name, ServiceStatus::STATUS_IN_PROGRESS);

	uint8_t failure_reason = 0;

	for(int tries = 0; tries < 10; ++tries)
	{
		if(m_fd < 0)
		{
			m_fd = socket(AF_INET, SOCK_STREAM, 0);
			if(m_fd < 0)
			{
				perror("Could not create socket");
				throw std::runtime_error("socket error");
			}

			if(connect(m_fd, (const sockaddr*)&m_addr, m_addrLen) != 0)
			{
				ROS_WARN("ServiceClient could not connect to server: %s", strerror(errno));
				sleep(1);

				close(m_fd);
				m_fd = -1;
				failure_reason = ServiceStatus::STATUS_CONNECTION_ERROR;
				continue;
			}

#ifdef TCP_USER_TIMEOUT
			int timeout = 8000;
			if(setsockopt(m_fd, SOL_TCP, TCP_USER_TIMEOUT, &timeout, sizeof(timeout)) != 0)
			{
				ROS_ERROR("Could not set TCP_USER_TIMEOUT: %s", strerror(errno));
				return false;
			}
#else
			ROS_WARN("Not setting TCP_USER_TIMEOUT");
#endif
		}

		try
		{
			sureWrite(m_fd, &req, sizeof(req));
			sureWrite(m_fd, name.c_str(), name.length());
			sureWrite(m_fd, &params.request.num_bytes, 4); // FIXME: Not portable
			sureWrite(m_fd, params.request.buf.get(), params.request.num_bytes);

			protocol::ServiceCallResponse resp;
			sureRead(m_fd, &resp, sizeof(resp));

			boost::shared_array<uint8_t> data(new uint8_t[resp.response_length()]);
			sureRead(m_fd, data.get(), resp.response_length());

			params.response = ros::SerializedMessage(data, resp.response_length());

			publishStatus(name, ServiceStatus::STATUS_FINISHED_SUCCESS);

			return true;
		}
		catch(IOException&)
		{
			failure_reason = ServiceStatus::STATUS_TIMEOUT;
			close(m_fd);
			m_fd = -1;
		}
	}

	publishStatus(name, failure_reason);
	return false;
}

void ServiceClient::publishStatus(const std::string& service, uint8_t status)
{
	ServiceStatus msg;
	msg.host = m_host;
	msg.remote = m_remote;
	msg.remote_port = m_remotePort;

	msg.call_id = m_currentCallID;
	msg.service = service;

	msg.status = status;

	m_pub_status.publish(msg);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_client");

	nimbro_service_transport::ServiceClient client;

	ros::spin();
	return 0;
}
