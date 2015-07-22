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

#include <nimbro_topic_transport/CompressedMsg.h>

#if WITH_PLOTTING
#  include <plot_msgs/Plot.h>
#endif

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

TopicData::TopicData()
 : m_decompressionThreadRunning(false)
{
}

TopicData::~TopicData()
{
	{
		boost::unique_lock<boost::mutex> lock(m_mutex);
		m_decompressionThreadShouldExit = true;
	}
	m_cond.notify_one();
}

void TopicData::takeForDecompression(const boost::shared_ptr<Message>& msg)
{
	if(!m_decompressionThreadRunning)
	{
		m_decompressionThreadShouldExit = false;
		m_decompressionThread = boost::thread(boost::bind(&TopicData::decompress, this));
		m_decompressionThreadRunning = true;
	}

	boost::unique_lock<boost::mutex> lock(m_mutex);
	m_compressedMsg = msg;
	m_cond.notify_one();
}

void TopicData::decompress()
{
	while(1)
	{
		boost::shared_ptr<Message> currentMessage;

		{
			boost::unique_lock<boost::mutex> lock(m_mutex);

			while(!m_compressedMsg && !m_decompressionThreadShouldExit)
				m_cond.wait(lock);

			if(m_decompressionThreadShouldExit)
				return;

			currentMessage = m_compressedMsg;
			m_compressedMsg.reset();
		}

		Message decompressed;
		currentMessage->decompress(&decompressed);

		topic_tools::ShapeShifter shapeShifter;
		shapeShifter.morph(md5_str, decompressed.header.topic_type, msg_def, "");

		shapeShifter.read(decompressed);

		publisher.publish(shapeShifter);
	}
}

UDPReceiver::UDPReceiver()
 : m_incompleteMessages(4)
{
	ros::NodeHandle nh("~");

	m_pub_heartbeat = nh.advertise<std_msgs::Time>("heartbeat", 1);

#if WITH_PLOTTING
	m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1000);
#endif

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

	nh.param("drop_repeated_msgs", m_dropRepeatedMessages, true);
	nh.param("warn_drop_incomplete", m_warnDropIncomplete, true);
	nh.param("keep_compressed", m_keepCompressed, false);

	nh.param("fec", m_fec, false);

#if !(WITH_RAPTORQ)
	if(m_fec)
		throw std::runtime_error("Please compile with RaptorQ support to enable FEC");
#endif
}

UDPReceiver::~UDPReceiver()
{
}

template<class HeaderType>
void UDPReceiver::handleFinishedMessage(Message* msg, HeaderType* header)
{
	// Packet complete

	// Enforce termination
	header->topic_type[sizeof(header->topic_type)-1] = 0;
	header->topic_name[sizeof(header->topic_name)-1] = 0;

	ROS_DEBUG("Got a packet of type %s, topic %s, (msg id %d), size %d", header->topic_type, header->topic_name, msg->id, (int)msg->payload.size());

	// Find topic
	TopicMap::iterator topic_it = m_topics.find(header->topic_name);

	boost::shared_ptr<TopicData> topic;
	if(topic_it == m_topics.end())
	{
		m_topics.insert(std::pair<std::string, boost::shared_ptr<TopicData>>(
			header->topic_name,
			boost::make_shared<TopicData>()
		));
		topic = m_topics[header->topic_name];

		topic->last_message_counter = -1;
	}
	else
		topic = topic_it->second;

	// Send heartbeat message
	ros::Time now = ros::Time::now();
	if(now - m_lastHeartbeatTime > ros::Duration(0.2))
	{
		std_msgs::Time time;
		time.data = now;
		m_pub_heartbeat.publish(time);

		m_lastHeartbeatTime = now;
	}

#if WITH_PLOTTING
	// Plot packet size
	plot_msgs::Plot plot;
	plot.header.stamp = ros::Time::now();

	plot_msgs::PlotPoint point;

	std::string safe_topic = header->topic_name;
	std::replace(safe_topic.begin(), safe_topic.end(), '/', '_');

	point.name = "/udp_receiver/mbyte/" + safe_topic;
	point.value = ((double)msg->getLength()) / 1024 / 1024;

	plot.points.push_back(point);
	m_pub_plot.publish(plot);
#endif

	if(m_dropRepeatedMessages && header->topic_msg_counter() == topic->last_message_counter)
	{
		// This is the same message, probably sent in relay mode
		return;
	}

	bool compressed = header->flags & UDP_FLAG_COMPRESSED;

	// Compare md5
	if(topic->last_message_counter == -1 || memcmp(topic->md5, header->topic_md5, sizeof(topic->md5)) != 0 || (m_keepCompressed && topic->compressed != compressed))
	{
		topic->msg_def = topic_info::getMsgDef(header->topic_type);
		topic->md5_str = topic_info::getMd5Sum(header->topic_type);

		ROS_WARN_STREAM("Got " << topic->msg_def << topic->md5_str << "end");
		for(int i = 0; i < 4; ++i)
		{
			std::string md5_part = topic->md5_str.substr(8*i, 8);
			uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
			topic->md5[i] = md5_num;
		}

		if(memcmp(topic->md5, header->topic_md5, sizeof(topic->md5)) != 0)
		{
			ROS_ERROR("Invalid md5 sum for topic type '%s', please make sure msg definitions are up to date", header->topic_type);
			return;
		}

		if(m_keepCompressed && compressed)
		{
			// If we are requested to keep the messages compressed, we advertise our compressed msg type
			topic->publisher = m_nh.advertise<CompressedMsg>(header->topic_name, 1);
		}
		else
		{
			// ... otherwise, we advertise the native type
			ros::AdvertiseOptions options(
				header->topic_name,
				1,
				topic->md5_str,
				header->topic_type,
				topic->msg_def
			);

			// Latching is often unexpected. Better to lose the first msg.
			//options.latch = 1;
			topic->publisher = m_nh.advertise(options);
		}

		topic->compressed = compressed;
	}

	if(compressed && m_keepCompressed)
	{
		CompressedMsg compressed;
		compressed.type = header->topic_type;
		memcpy(compressed.md5.data(), topic->md5, sizeof(topic->md5));

		compressed.data.swap(msg->payload);
		topic->publisher.publish(compressed);
	}
	else if(header->flags & UDP_FLAG_COMPRESSED)
		topic->takeForDecompression(boost::make_shared<Message>(*msg));
	else
	{
		topic_tools::ShapeShifter shapeShifter;
		shapeShifter.morph(topic->md5_str, header->topic_type, topic->msg_def, "");

		shapeShifter.read(*msg);

		topic->publisher.publish(shapeShifter);
	}

	topic->last_message_counter = header->topic_msg_counter();
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
		uint16_t msg_id;

		if(m_fec)
		{
			FECPacket::Header* header = (FECPacket::Header*)buf;
			msg_id = header->msg_id();
		}
		else
		{
			UDPGenericPacket* generic = (UDPGenericPacket*)buf;
			msg_id = generic->msg_id();
		}

		MessageBuffer::iterator it = std::find_if(m_incompleteMessages.begin(), m_incompleteMessages.end(),
			[=](const Message& msg) { return msg.id == msg_id; }
		);

		if(it == m_incompleteMessages.end())
		{
			// Insert a new message
			m_incompleteMessages.push_front(Message(msg_id));
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

			if(m_warnDropIncomplete)
			{
				for(MessageBuffer::iterator itd = itr; itd != it_end; ++itd)
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
			}

			m_incompleteMessages.erase(itr, it_end);
		}

		msg = &*it;

		if(m_fec)
		{
#if WITH_RAPTORQ
			FECPacket* packet = (FECPacket*)buf;

			if(!msg->decoder)
			{
				msg->decoder.reset(new Message::Decoder(
					packet->header.oti_common(), packet->header.oti_specific()
				));
			}

			uint8_t* symbol_begin = packet->data;
			msg->decoder->add_symbol(symbol_begin, buf + size, packet->header.symbol_id());

			// Try decoding
			msg->payload.resize(msg->decoder->bytes());
			auto outIt = msg->payload.begin();
			uint64_t written = msg->decoder->decode(outIt, msg->payload.end());

			if(written != 0)
			{
				if(written < sizeof(FECHeader))
				{
					ROS_ERROR("Invalid short packet");
					m_incompleteMessages.erase(it);
					continue;
				}

				FECHeader msgHeader = *reinterpret_cast<FECHeader*>(msg->payload.data());

				// Remove header from buffer
				memmove(msg->payload.data(), msg->payload.data() + sizeof(FECHeader), written - sizeof(FECHeader));
				msg->payload.resize(written - sizeof(FECHeader));

				msg->size = written;

				handleFinishedMessage(msg, &msgHeader);
				m_incompleteMessages.erase(it);
			}
#endif
		}
		else
		{
			UDPGenericPacket* generic = (UDPGenericPacket*)buf;
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

			ROS_DEBUG("fragment: %d of msg %d", (int)generic->frag_id(), (int)msg_id);
			msg->msgs[generic->frag_id] = true;

			if(std::all_of(msg->msgs.begin(), msg->msgs.end(), [](bool x){return x;}))
			{
				handleFinishedMessage(msg, &msg->header);

				m_incompleteMessages.erase(it);
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
