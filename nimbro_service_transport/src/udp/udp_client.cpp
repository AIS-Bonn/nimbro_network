// UDP service client
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_client.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <netdb.h>

#include <sstream>

#include <ros/names.h>
#include <ros/package.h>

#include <nimbro_service_transport/ServiceStatus.h>

#include "protocol.h"
#include "../common.h"

namespace nimbro_service_transport
{

namespace
{

class CallbackHelper : public ros::ServiceCallbackHelper
{
public:
	CallbackHelper(const std::string& name, UDPClient* client)
	 : m_name(name)
	 , m_client(client)
	{}

	virtual bool call(ros::ServiceCallbackHelperCallParams& params)
	{
		return m_client->call(m_name, params);
	}
private:
	std::string m_name;
	UDPClient* m_client;
};

}


UDPClient::UDPClient()
 : m_nh("~")
 , m_fd(-1)
 , m_counter(0)
{
	m_nh.param("port", m_remotePort, 5000);

	char portString[100];
	snprintf(portString, sizeof(portString), "%d", m_remotePort);

	if(!m_nh.getParam("server", m_remote))
		throw std::runtime_error("udp_client requires the 'server' parameter");

	// Get local host name for visualization messages
	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_host = hostnameBuf;

	m_nh.param("timeout", m_timeout, 5.0);

	// Resolve remote address
	addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_socktype = SOCK_DGRAM;

	addrinfo* info;

	if(getaddrinfo(m_remote.c_str(), portString, &hints, &info) != 0 || !info)
	{
		std::stringstream ss;
		ss << "getaddrinfo() failed for host '" << m_remote << "': " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	m_fd = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
	if(m_fd < 0)
	{
		std::stringstream ss;
		ss << "Could not create socket: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	if(connect(m_fd, info->ai_addr, info->ai_addrlen) != 0)
	{
		std::stringstream ss;
		ss << "Could not connect socket: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	freeaddrinfo(info);

	// Initialize & advertise the list of services
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

UDPClient::~UDPClient()
{
	close(m_fd);
}

uint8_t UDPClient::acquireCounterValue()
{
	boost::unique_lock<boost::mutex> lock(m_mutex);
	return m_counter++;
}

bool UDPClient::call(const std::string& name, ros::ServiceCallbackHelperCallParams& params)
{
	std::vector<uint8_t> buffer(sizeof(ServiceCallRequest) + name.length() + 4 + params.request.num_bytes);

	ServiceCallRequest* header = reinterpret_cast<ServiceCallRequest*>(buffer.data());
	header->timestamp = ros::Time::now().toNSec();
	header->counter = acquireCounterValue();
	header->name_length = name.length();
	header->request_length = params.request.num_bytes + 4;

	memcpy(buffer.data() + sizeof(ServiceCallRequest), name.c_str(), name.length());
	memcpy(buffer.data() + sizeof(ServiceCallRequest) + name.length(), &params.request.num_bytes, 4);
	memcpy(buffer.data() + sizeof(ServiceCallRequest) + name.length() + 4, params.request.buf.get(), params.request.num_bytes);

	RequestRecord record;
	record.timestamp = header->timestamp();
	record.counter = header->counter;
	record.response.num_bytes = 0;

	boost::unique_lock<boost::mutex> lock(m_mutex);

	publishStatus(name, header->counter, ServiceStatus::STATUS_IN_PROGRESS);

	auto it = m_requests.insert(m_requests.end(), &record);

	if(send(m_fd, buffer.data(), buffer.size(), 0) != (int)buffer.size())
	{
		ROS_ERROR("Could not send UDP data: %s", strerror(errno));
		publishStatus(name, header->counter, ServiceStatus::STATUS_CONNECTION_ERROR);
		return false;
	}

	// TODO: Timeout
	boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(m_timeout * 1000);

	bool gotMsg = record.cond.timed_wait(lock, timeout, [&](){ return record.response.num_bytes != 0; });
	m_requests.erase(it);

	if(gotMsg)
	{
		params.response = record.response;
		publishStatus(name, header->counter, ServiceStatus::STATUS_FINISHED_SUCCESS);
		return true;
	}
	else
	{
		ROS_WARN("timeout!");
		publishStatus(name, header->counter, ServiceStatus::STATUS_TIMEOUT);
		return false;
	}
}

void UDPClient::step()
{
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 500 * 1000;

	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(m_fd, &fds);

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
}

void UDPClient::handlePacket()
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

		m_recvBuf.resize(packetSize);
		break;
	}

	int bytes = recv(m_fd, m_recvBuf.data(), m_recvBuf.size(), 0);
	if(bytes < 0)
	{
		if(errno == ECONNREFUSED)
		{
			ROS_ERROR("Got negative ICMP reply. Is the UDP server running?");
			return;
		}

		std::stringstream ss;
		ss << "Could not recv(): " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	if(bytes < (int)sizeof(ServiceCallResponse))
	{
		ROS_ERROR("Short response packet, ignoring...");
		return;
	}

	const auto* resp = reinterpret_cast<const ServiceCallResponse*>(m_recvBuf.data());

	if(resp->response_length() + sizeof(ServiceCallResponse) > (size_t)bytes)
	{
		ROS_ERROR("Response is longer than packet...");
		return;
	}

	boost::unique_lock<boost::mutex> lock(m_mutex);
	for(auto it = m_requests.begin(); it != m_requests.end(); ++it)
	{
		if(resp->timestamp() == (*it)->timestamp && resp->counter == (*it)->counter)
		{
			boost::shared_array<uint8_t> data(new uint8_t[resp->response_length()]);
			memcpy(data.get(), m_recvBuf.data() + sizeof(ServiceCallResponse), resp->response_length());
			(*it)->response = ros::SerializedMessage(data, resp->response_length());

			(*it)->cond.notify_one();
			return;
		}
	}

	ROS_ERROR("Received unexpected UDP service packet answer, ignoring");
}

void UDPClient::publishStatus(const std::string& service, uint32_t call, uint8_t status)
{
	ServiceStatus msg;
	msg.host = m_host;
	msg.remote = m_remote;
	msg.remote_port = m_remotePort;

	msg.call_id = call;
	msg.service = service;

	msg.status = status;

	m_pub_status.publish(msg);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_client");

	nimbro_service_transport::UDPClient client;

	ros::AsyncSpinner spinner(10);
	spinner.start();

	while(ros::ok())
	{
		client.step();
	}

	spinner.stop();

	return 0;
}
