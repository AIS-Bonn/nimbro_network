// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_receiver.h"
#include "udp_packet.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <topic_tools/shape_shifter.h>

#include <stdio.h>

namespace sb_udp
{

static std::string getMsgDef(const std::string& type)
{
	std::vector<char> buf(1024);
	int idx = 0;

	// FIXME: This is fricking dangerous!
	FILE* f = popen(("rosmsg show \'" + type + "\'").c_str(), "r");

	while(!feof(f))
	{
		buf.resize(idx + 1024);
		size_t size = fread(buf.data() + idx, 1, 1024, f);
		if(size == 0)
			break;

		idx += size;
	}

	int exit_code = pclose(f);

	if(exit_code != 0)
	{
		ROS_WARN("Could not get msg def for type '%s'", type.c_str());
		return "";
	}
	else
	{
		return std::string(buf.data(), idx);
	}
}

static std::string getMd5Sum(const std::string& type)
{
	std::vector<char> buf(1024);
	int idx = 0;

	// FIXME: This is fricking dangerous!
	FILE* f = popen(("rosmsg md5 \'" + type + "\'").c_str(), "r");

	while(!feof(f))
	{
		buf.resize(idx + 1024);
		size_t size = fread(buf.data() + idx, 1, 1024, f);
		if(size == 0)
			break;

		idx += size;
	}

	int exit_code = pclose(f);

	if(exit_code != 0)
	{
		fprintf(stderr, "Could not get md5 sum for type '%s'\n", type.c_str());
		return "";
	}
	else
	{
		std::string ret(buf.data(), idx-1);
		return ret;
	}
}

UDPReceiver::UDPReceiver()
 : m_incompleteMessages(4)
{
	ros::NodeHandle nh("~");

	m_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		ROS_FATAL("Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int port;
	nh.param("port", port, 5050);

	std::string baddr;
	nh.param("address", baddr, std::string("192.168.178.255"));

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(baddr.c_str());
	addr.sin_port = htons(port);

	ROS_INFO("Binding to '%s':%d", baddr.c_str(), port);

	if(bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		ROS_FATAL("Could not bind socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
	{
		ROS_FATAL("Could not set broadcast flag: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}
}

UDPReceiver::~UDPReceiver()
{
}

void UDPReceiver::run()
{
	uint8_t buf[PACKET_SIZE];

	ROS_INFO("ready");
	while(1)
	{
		ros::spinOnce();

		ssize_t size = recv(m_fd, buf, sizeof(buf), 0);

		if(size < 0)
		{
			ROS_FATAL("Could not recv(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

// 		ROS_WARN("packet");

		UDPGenericPacket* generic = (UDPGenericPacket*)buf;

		MessageBuffer::iterator it = std::find_if(m_incompleteMessages.begin(), m_incompleteMessages.end(),
			[=](const Message& msg) { return msg.valid && msg.id == generic->msg_id(); }
		);

		Message* msg;
		if(it == m_incompleteMessages.end())
		{
			// New message
			m_incompleteMessages.push_front(Message(generic->msg_id()));
			msg = &(*m_incompleteMessages.begin());
		}
		else
			msg = &*it;

		if(generic->frag_id == 0)
		{
			UDPFirstPacket* first = (UDPFirstPacket*)buf;

			msg->header = first->header;

			// We can calculate an approximate size now
			uint32_t required_size = (msg->header.remaining_packets()+1) * PACKET_SIZE;
			uint32_t my_size = size - sizeof(UDPFirstPacket);
			if(msg->payload.size() < required_size)
				msg->payload.resize(required_size);
			memcpy(msg->payload.data(), first->data, my_size);

			if(msg->size < my_size)
				msg->size = my_size;

			if(((uint16_t)msg->msgs.size()) < msg->header.remaining_packets()+1)
				msg->msgs.resize(msg->header.remaining_packets()+1, false);
		}
		else
		{
			UDPDataPacket* data = (UDPDataPacket*)buf;

			uint32_t offset = UDPFirstPacket::MaxDataSize + (data->header.frag_id-1) * UDPDataPacket::MaxDataSize;
			uint32_t required_size = offset + size - sizeof(UDPDataPacket);
			if(msg->payload.size() < required_size)
				msg->payload.resize(required_size);
			memcpy(msg->payload.data() + offset, data->data, size - sizeof(UDPDataPacket));

			if(msg->size < required_size)
				msg->size = required_size;
		}

		if(generic->frag_id >= msg->msgs.size())
			msg->msgs.resize(generic->frag_id+1, false);
		msg->msgs[generic->frag_id] = true;

		if(std::all_of(msg->msgs.begin(), msg->msgs.end(), [](bool x){return x;}))
		{
			// Packet complete

			// Enforce termination
			msg->header.topic_type[sizeof(msg->header.topic_type)-1] = 0;
			msg->header.topic_name[sizeof(msg->header.topic_name)-1] = 0;

			ROS_WARN("Got a packet of type %s, topic %s, %d extra udp packets (msg id %d)", msg->header.topic_type, msg->header.topic_name, msg->header.remaining_packets(), msg->id);

			// Find topic
			TopicMap::iterator it = m_topics.find(msg->header.topic_type);

			TopicData* topic;
			if(it == m_topics.end())
			{
				m_topics.insert(std::pair<std::string, TopicData>(
					msg->header.topic_name,
					TopicData()
				));
				topic = &m_topics[msg->header.topic_name];
			}
			else
				topic = &it->second;

			// Compare md5
			if(memcmp(topic->md5, msg->header.topic_md5, sizeof(topic->md5)) != 0)
			{
				topic->msg_def = getMsgDef(msg->header.topic_type);
				topic->md5_str = getMd5Sum(msg->header.topic_type);

				ROS_WARN_STREAM("Got " << topic->msg_def << topic->md5_str << "end");
				for(int i = 0; i < 4; ++i)
				{
					std::string md5_part = topic->md5_str.substr(4*i, 4);
					uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
					topic->md5[i] = md5_num;
				}

				if(memcmp(topic->md5, msg->header.topic_md5, sizeof(topic->md5)) != 0)
				{
					ROS_ERROR("Invalid md5 sum for topic type '%s', please make sure msg definitions are up to date", msg->header.topic_type);
					msg->valid = false;
					continue;
				}

				ros::AdvertiseOptions options(
					msg->header.topic_name,
					1,
					topic->md5_str,
					msg->header.topic_type,
					topic->msg_def
				);
				topic->publisher = m_nh.advertise(options);
			}

			topic_tools::ShapeShifter shapeShifter;

			shapeShifter.morph(topic->md5_str, msg->header.topic_type, topic->msg_def, "");
			shapeShifter.read(*msg);

			topic->publisher.publish(shapeShifter);

			msg->valid = false;
		}
	}
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_receiver", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	sb_udp::UDPReceiver recv;
	recv.run();

	return 0;
}
