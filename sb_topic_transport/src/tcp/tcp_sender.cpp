// TCP sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tcp_sender.h"
#include "../topic_info.h"

#include <bzlib.h>

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
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		std::string topic = list[i]["name"];
		int flags = 0;

		if(list[i].hasMember("compress") && ((bool)list[i]["compress"]) == true)
			flags |= TCP_FLAG_COMPRESSED;

		boost::function<void(const topic_tools::ShapeShifter&)> func;
		func = boost::bind(&TCPSender::send, this, topic, flags, _1);

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

void TCPSender::send(const std::string& topic, int flags, const topic_tools::ShapeShifter& shifter)
{
	std::string type = shifter.getDataType();
	std::string md5 = shifter.getMD5Sum();
	uint32_t size = shifter.size();

	uint32_t maxDataSize = size;

	if(flags & TCP_FLAG_COMPRESSED)
		maxDataSize = size + size / 100 + 1200; // taken from bzip2 docs

	m_packet.resize(
		sizeof(TCPHeader) + topic.length() + type.length() + maxDataSize
	);

	TCPHeader* header = (TCPHeader*)m_packet.data();

	// Fill in topic & type
	uint8_t* wptr = m_packet.data() + sizeof(TCPHeader);

	memcpy(wptr, topic.c_str(), topic.length());
	wptr += topic.length();

	memcpy(wptr, type.c_str(), type.length());
	wptr += type.length();

	if(flags & TCP_FLAG_COMPRESSED)
	{
		unsigned int len = m_packet.size() - (wptr - m_packet.data());

		m_compressionBuf.resize(shifter.size());
		PtrStream stream(m_compressionBuf.data());
		shifter.write(stream);

		if(BZ2_bzBuffToBuffCompress((char*)wptr, &len, (char*)m_compressionBuf.data(), m_compressionBuf.size(), 7, 0, 30) == BZ_OK)
		{
			header->data_len = len;
			wptr += len;
			size = len;
		}
		else
		{
			ROS_ERROR("Could not compress with bzip2 library, sending uncompressed");
			flags &= ~TCP_FLAG_COMPRESSED;
			memcpy(wptr, m_compressionBuf.data(), m_compressionBuf.size());
			header->data_len = m_compressionBuf.size();
			wptr += m_compressionBuf.size();
		}
	}
	else
	{
		PtrStream stream(wptr);
		shifter.write(stream);
		header->data_len = size;
		wptr += size;
	}

	header->topic_len = topic.length();
	header->type_len = type.length();
	header->data_len = size;
	header->flags = flags;
	topic_info::packMD5(md5, header->topic_md5sum);

	// Resize to final size
	m_packet.resize(
		wptr - m_packet.data()
	);

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

		if(write(m_fd, m_packet.data(), m_packet.size()) != (int)m_packet.size())
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

