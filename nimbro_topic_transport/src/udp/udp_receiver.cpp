// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_receiver.h"
#include "udp_packet.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <topic_tools/shape_shifter.h>

#include <std_msgs/Time.h>

#include <stdio.h>

#include <ros/names.h>

#include "../topic_info.h"

#include <bzlib.h>

namespace nimbro_topic_transport
{

bool Message::decompress(Message* dest)
{
	unsigned int destLen = 1024;
	dest->payload.resize(destLen);

	while(1)
	{
		int ret = BZ2_bzBuffToBuffDecompress((char*)dest->payload.data(), &destLen, (char*)payload.data(), payload.size(), 0, 0);

		if(ret == BZ_OUTBUFF_FULL)
		{
			destLen *= 2;
			dest->payload.resize(destLen);
			continue;
		}

		if(ret != BZ_OK)
		{
			ROS_ERROR("Could not decompress message");
			return false;
		}

		break;
	}

	dest->payload.resize(destLen);
	dest->header = header;
	dest->id = id;
	dest->size = destLen;
	return true;
}


UDPReceiver::UDPReceiver()
 : m_incompleteMessages(4)
{
	ros::NodeHandle nh("~");

	m_pub_heartbeat = nh.advertise<std_msgs::Time>("heartbeat", 1);

	m_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		ROS_FATAL("Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int port;
	nh.param("port", port, 5050);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	ROS_INFO("Binding to :%d", port);

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

	nh.param("drop_repeated_msgs", m_dropRepeatedMessages, false);
}

UDPReceiver::~UDPReceiver()
{
}

void UDPReceiver::run()
{
	uint8_t buf[PACKET_SIZE];

	ROS_INFO("UDP receiver ready");
	while(1)
	{
		ros::spinOnce();

		ssize_t size = recv(m_fd, buf, sizeof(buf), 0);

		if(size < 0)
		{
			ROS_FATAL("Could not recv(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		ROS_DEBUG("packet");

		Message* msg;

		UDPGenericPacket* generic = (UDPGenericPacket*)buf;

		MessageBuffer::iterator it = std::find_if(m_incompleteMessages.begin(), m_incompleteMessages.end(),
			[=](const Message& msg) { return msg.id == generic->msg_id(); }
		);

		if(it == m_incompleteMessages.end())
		{
			// Insert a new message
			m_incompleteMessages.push_front(Message(generic->msg_id()));
			it = m_incompleteMessages.begin();

			// Erase messages that are too old (after index 31)
			MessageBuffer::iterator itr = m_incompleteMessages.begin();
			MessageBuffer::iterator it_end = m_incompleteMessages.end();
			for(int i = 0; i < 32; ++i)
			{
				itr++;
				if(itr == it_end)
					break;
			}

			for(MessageBuffer::iterator itd = itr; itr != it_end; ++itr)
			{
				const Message& msg = *itd;

				int num_fragments = msg.msgs.size();
				int received = 0;
				for(unsigned int i = 0; i < msg.msgs.size(); ++i)
				{
					if(msg.msgs[i])
						received++;
				}

				ROS_WARN("Dropping message %d, %.2f%% of fragments received (%d/%d)",
					msg.id, 100.0 * received / num_fragments, received, num_fragments
				);
			}

			m_incompleteMessages.erase(itr, it_end);
		}

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

		ROS_DEBUG("fragment: %d of msg %d", (int)generic->frag_id(), (int)generic->msg_id());
		msg->msgs[generic->frag_id] = true;

		if(std::all_of(msg->msgs.begin(), msg->msgs.end(), [](bool x){return x;}))
		{
			// Packet complete

			// Enforce termination
			msg->header.topic_type[sizeof(msg->header.topic_type)-1] = 0;
			msg->header.topic_name[sizeof(msg->header.topic_name)-1] = 0;

			ROS_DEBUG("Got a packet of type %s, topic %s, %d extra udp packets (msg id %d)", msg->header.topic_type, msg->header.topic_name, msg->header.remaining_packets(), msg->id);

			// Find topic
			TopicMap::iterator topic_it = m_topics.find(msg->header.topic_name);

			TopicData* topic;
			if(topic_it == m_topics.end())
			{
				m_topics.insert(std::pair<std::string, TopicData>(
					msg->header.topic_name,
					TopicData()
				));
				topic = &m_topics[msg->header.topic_name];

				topic->last_message_counter = -1;
			}
			else
				topic = &topic_it->second;

			if(m_dropRepeatedMessages && msg->header.topic_msg_counter() == topic->last_message_counter)
			{
				// This is the same message, probably sent in relay mode
				m_incompleteMessages.erase(it);
				continue;
			}

			// Compare md5
			if(memcmp(topic->md5, msg->header.topic_md5, sizeof(topic->md5)) != 0)
			{
				topic->msg_def = topic_info::getMsgDef(msg->header.topic_type);
				topic->md5_str = topic_info::getMd5Sum(msg->header.topic_type);

				ROS_WARN_STREAM("Got " << topic->msg_def << topic->md5_str << "end");
				for(int i = 0; i < 4; ++i)
				{
					std::string md5_part = topic->md5_str.substr(8*i, 8);
					uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
					topic->md5[i] = md5_num;
				}

				if(memcmp(topic->md5, msg->header.topic_md5, sizeof(topic->md5)) != 0)
				{
					ROS_ERROR("Invalid md5 sum for topic type '%s', please make sure msg definitions are up to date", msg->header.topic_type);
					m_incompleteMessages.erase(it);
					continue;
				}

				ros::AdvertiseOptions options(
					msg->header.topic_name,
					1,
					topic->md5_str,
					msg->header.topic_type,
					topic->msg_def
				);

				// Latching is often unexpected. Better to lose the first msg.
// 				options.latch = 1;
				topic->publisher = m_nh.advertise(options);
			}

			topic_tools::ShapeShifter shapeShifter;

			shapeShifter.morph(topic->md5_str, msg->header.topic_type, topic->msg_def, "");

			if(msg->header.flags & UDP_FLAG_COMPRESSED)
			{
				if(!msg->decompress(&m_decompressedMessage))
				{
					ROS_ERROR("Could not decompress message, dropping");
					continue;
				}
				shapeShifter.read(m_decompressedMessage);
			}
			else
				shapeShifter.read(*msg);

			topic->publisher.publish(shapeShifter);

			topic->last_message_counter = msg->header.topic_msg_counter();

			m_incompleteMessages.erase(it);

			// Send heartbeat message
			ros::Time now = ros::Time::now();
			if(now - m_lastHeartbeatTime > ros::Duration(1.0))
			{
				std_msgs::Time time;
				time.data = now;
				m_pub_heartbeat.publish(time);

				m_lastHeartbeatTime = now;
			}
		}
	}
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "udp_receiver", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	nimbro_topic_transport::UDPReceiver recv;
	recv.run();

	return 0;
}
