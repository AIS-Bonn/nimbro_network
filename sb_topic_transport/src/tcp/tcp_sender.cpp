// TCP sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_sender.h"
#include "../topic_info.h"

namespace sb_topic_transport
{

TCPSender::TCPSender()
 : m_nh("~")
 , m_fd(-1)
{
	std::string addr;
	if(!m_nh.getParam("address", addr))
	{
		ROS_FATAL("tcp_sender needs an 'address' parameter!");
		throw std::runtime_error("tcp_sender needs an 'address' parameter!");
	}

	int port;
	if(!m_nh.getParam("port", port))
	{
		ROS_FATAL("tcp_sender needs a 'port' parameter!");
		throw std::runtime_error("tcp_sender needs a 'port' parameter!");
	}

	memset(&m_addr, 0, sizeof(m_addr));
	m_addr.sin_family = AF_INET;
	m_addr.sin_addr.s_addr = inet_addr(addr.c_str());
	m_addr.sin_port = htons(port);

	XmlRpc::XmlRpcValue list;
	m_nh.getParam("topics", list);

	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int32_t i = 0; i < list.size(); ++i)
	{
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);

		boost::function<void(const topic_tools::ShapeShifter&)> func;
		func = boost::bind(&TCPSender::send, this, (std::string)list[i], _1);

		m_subs.push_back(
			m_nh.subscribe<const topic_tools::ShapeShifter>((std::string)list[i], 20, func)
		);
	}
}

TCPSender::~TCPSender()
{
}

bool TCPSender::connect()
{
	m_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(m_fd < 0)
	{
		ROS_ERROR("Could not create socket: %s", strerror(errno));
		return false;
	}

	if(::connect(m_fd, (sockaddr*)&m_addr, sizeof(m_addr)) != 0)
	{
		ROS_ERROR("Could not connect: %s", strerror(errno));
		return false;
	}

	return true;
}

class PtrStream
{
public:
	PtrStream(uint8_t* ptr)
	 : m_ptr(ptr)
	{}

	inline uint8_t* advance(int)
	{ return m_ptr; }
private:
	uint8_t* m_ptr;
};

static void push(const void* data, size_t size, std::vector<uint8_t>* dest)
{
	int pos = dest->size();
	dest->resize(pos + size);
	memcpy(dest->data() + pos, data, size);
}

void TCPSender::send(const std::string& topic, const topic_tools::ShapeShifter& shifter)
{
	std::string type = shifter.getDataType();
	std::string md5 = shifter.getMD5Sum();
	uint32_t size = shifter.size();

	std::vector<uint8_t> packet;
	packet.reserve(
		sizeof(TCPHeader) + topic.length() + type.length() + size
	);

	// Header
	TCPHeader header;
	header.topic_len = topic.length();
	header.type_len = type.length();
	header.data_len = size;

	topic_info::packMD5(md5, header.topic_md5sum);

	ROS_DEBUG("Sending header with topic_len %d, type_len %d, data_len %d", header.topic_len(), header.type_len(), header.data_len());
	push(&header, sizeof(header), &packet);

	// Topic name
	push(topic.c_str(), topic.length(), &packet);

	// Type name
	push(type.c_str(), type.length(), &packet);

	// Data
	uint32_t pos = packet.size();
	packet.resize(pos + size);
	PtrStream stream(packet.data() + pos);
	shifter.write(stream);

	// Try to send the packet
	for(int tries = 0; tries < 10; ++tries)
	{
		if(m_fd == -1)
		{
			if(!connect())
			{
				ROS_WARN("Connection failed, trying again");
				continue;
			}
		}

		if(write(m_fd, packet.data(), packet.size()) != (int)packet.size())
		{
			ROS_WARN("Could not send data, trying again");
			close(m_fd);
			m_fd = -1;
			continue;
		}

		// Read ACK
		uint8_t ack;
		if(read(m_fd, &ack, 1) != 1)
		{
			ROS_WARN("Could not read ACK, sending again (!)");
			close(m_fd);
			m_fd = -1;
			continue;
		}

		return;
	}

	ROS_ERROR("Could not send TCP packet. Dropping message from topic %s!", topic.c_str());
}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tcp_sender");

	sb_topic_transport::TCPSender sender;

	ros::spin();

	return 0;
}

