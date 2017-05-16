// Handles one client in a dedicated thread
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "client_handler.h"
#include "protocol.h"

#include <ros/console.h>
#include <ros/master.h>
#include <ros/service_manager.h>
#include <ros/poll_manager.h>
#include <ros/connection.h>
#include <ros/service.h>
#include <ros/transport/transport_tcp.h>

#include <topic_tools/shape_shifter.h>

#define DATA_DEBUG 0

namespace nimbro_service_transport
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

ClientHandler::ClientHandler(int fd)
 : m_fd(fd)
 , m_thread(boost::bind(&ClientHandler::run, this))
{
}

ClientHandler::~ClientHandler()
{
	close(m_fd);
}

void ClientHandler::run()
{
	while(1)
	{
		protocol::ServiceCallRequest request;
		if(sureRead(m_fd, &request, sizeof(request)) <= 0)
		{
			ROS_WARN("wonky, closing connection");
			close(m_fd);
			return;
		}

		if(request.name_length() == 0)
		{
			sendAvailableServices();
		}
		else
		{
			std::string service(request.name_length(), ' ');
			sureRead(m_fd, (uint8_t*)service.data(), request.name_length());

			ros::ServiceClientOptions ops(service, "*", false, ros::M_string());
			ros::ServiceClient client = ros::NodeHandle().serviceClient(ops);

			boost::shared_array<uint8_t> array(new uint8_t[request.request_length()]);
			sureRead(m_fd, (uint8_t*)array.get(), request.request_length());
			ros::SerializedMessage msg_request(array, request.request_length());
			ros::SerializedMessage msg_response;

#if DATA_DEBUG
			ROS_INFO("Calling service '%s' with %d bytes of request data", service.c_str(), request.request_length());
			for(int i = 0; i < msg_request.num_bytes; ++i)
				ROS_INFO(" %d: 0x%02X (%c)", i, msg_request.buf.get()[i], msg_request.buf.get()[i]);
#endif

			bool ok = client.call(msg_request, msg_response, std::string("*"));

			topic_tools::ShapeShifter response;
			ros::serialization::deserializeMessage(msg_response, response);

			ros::SerializedMessage msg_service_response;

			msg_service_response = ros::serialization::serializeServiceResponse(ok, response);

			protocol::ServiceCallResponse resp;
			resp.response_length = msg_service_response.num_bytes;
			sureWrite(m_fd, &resp, sizeof(resp));
			sureWrite(m_fd, msg_service_response.buf.get(), msg_service_response.num_bytes);
		}
	}
}

static void pushIntoVector(std::vector<uint8_t>* dest, const void* data, size_t size)
{
	dest->resize(dest->size() + size);
	std::vector<uint8_t>::iterator it = dest->end() - size;
	std::copy((const uint8_t*)data, (const uint8_t*)data + size, it);
}

void ClientHandler::sendAvailableServices()
{
	std::vector<uint8_t> response;

	XmlRpc::XmlRpcValue systemState;
	XmlRpc::XmlRpcValue payload;
	if(!ros::master::execute("getSystemState", "service_server", systemState, payload, true))
		throw std::runtime_error("Could not get system state from ROS master");

	if(systemState.getType() != XmlRpc::XmlRpcValue::TypeArray)
		throw std::runtime_error("Invalid rpc answer 1");

	XmlRpc::XmlRpcValue state = systemState[2];

	if(state.getType() != XmlRpc::XmlRpcValue::TypeArray)
		throw std::runtime_error("Invalid rpc answer 2");

	XmlRpc::XmlRpcValue services = state[2];

	if(state.getType() != XmlRpc::XmlRpcValue::TypeArray)
		throw std::runtime_error("Invalid rpc answer 3");

	for(int i = 0; i < services.size(); ++i)
	{
		XmlRpc::XmlRpcValue service = services[i];

		if(service.getType() != XmlRpc::XmlRpcValue::TypeArray)
			throw std::runtime_error("Invalid rpc answer 4");

		if(service[0].getType() != XmlRpc::XmlRpcValue::TypeString)
			continue;

		std::string name = service[0];
		m_gotHeader = false;

		if(name.length() > 9 && name.substr(0, 8) == "/service")
			continue;

		std::string host;
		uint32_t port;

		if(!ros::ServiceManager::instance()->lookupService(name, host, port))
		{
			ROS_WARN("Could not lookup service '%s', ignoring...", name.c_str());
			continue;
		}

		ros::TransportTCPPtr tcp(new ros::TransportTCP(&ros::PollManager::instance()->getPollSet()));
		if(!tcp->connect(host, port))
		{
			ROS_WARN("Could not connect to service '%s', ignoring...", name.c_str());
		}

		ros::ConnectionPtr connection(new ros::Connection());
		connection->initialize(tcp, false, boost::bind(&ClientHandler::handleHeader, this, _1, _2));

		ros::M_string header;
		header["probe"] = "1";
		header["md5sum"] = "*";
		header["callerid"] = "service_manager";
		header["service"] = name;

		connection->writeHeader(header, boost::bind(&ClientHandler::handleHeaderWritten, this));

		while(!connection->isDropped())
		{
			ros::spinOnce();
			usleep(10*1000);
		}

		tcp->close();

		if(!m_gotHeader)
		{
			ROS_WARN("Could not connect to service '%s', ignoring...", name.c_str());
			continue;
		}

		ROS_DEBUG("service: %s", name.c_str());
		boost::shared_ptr<ros::M_string> values = m_headerBuf.getValues();
		for(ros::M_string::iterator it = values->begin(); it != values->end(); ++it)
		{
			ROS_DEBUG(" header %s: %s", it->first.c_str(), it->second.c_str());
		}

		protocol::ServiceDescription desc;
		std::string md5sum, type;
		if(!m_headerBuf.getValue("md5sum", md5sum))
		{
			ROS_ERROR("missing md5sum on service '%s'", name.c_str());
			continue;
		}
		if(!m_headerBuf.getValue("type", type))
		{
			ROS_ERROR("missing type on service '%s'", name.c_str());
			continue;
		}

		for(unsigned int i = 0; i < 4; ++i)
		{
			if(md5sum.length() < 8*i + 8)
			{
				ROS_ERROR("wonky md5sum: '%s'", md5sum.c_str());
				break;
			}
			std::string md5_part = md5sum.substr(8*i, 8);
			uint32_t md5_num = strtoll(md5_part.c_str(), 0, 16);
			desc.md5[i] = md5_num;
		}

		desc.name_length = name.length();
		desc.type_length = type.length();
		
		ROS_INFO("service '%s'", name.c_str());

		pushIntoVector(&response, &desc, sizeof(desc));
		pushIntoVector(&response, name.c_str(), name.length());
		pushIntoVector(&response, type.c_str(), type.length());
	}

	protocol::ServiceCallResponse resp;
	resp.response_length = response.size();

	sureWrite(m_fd, &resp, sizeof(resp));
	sureWrite(m_fd, response.data(), response.size());
}

bool ClientHandler::handleHeader(const ros::ConnectionPtr& connection, const ros::Header& header)
{
	m_headerBuf = header;
	m_gotHeader = true;
	connection->drop(ros::Connection::TransportDisconnect);
	return true;
}

void ClientHandler::handleHeaderWritten()
{
}

}
