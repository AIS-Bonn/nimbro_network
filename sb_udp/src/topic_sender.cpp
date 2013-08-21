// Sends a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_sender.h"
#include "udp_sender.h"
#include "udp_packet.h"

namespace sb_udp
{

TopicSender::TopicSender(UDPSender* sender, ros::NodeHandle* nh, const std::string& topic, double rate)
 : m_sender(sender)
{
	ROS_INFO_STREAM("Subscribing to" << topic);
	m_subscriber = nh->subscribe(topic, 1, &TopicSender::handleData, this);
	m_topicName = topic;

	m_durationBetweenPackets = ros::Duration(1.0 / rate);
}

void TopicSender::handleData(const topic_tools::ShapeShifter& shapeShifter)
{
	ros::Time now = ros::Time::now();
	if(now - m_lastTime < m_durationBetweenPackets)
		return;

	m_lastTime = now;

	uint8_t buf[PACKET_SIZE];
	uint32_t buf_size = std::min<uint32_t>(PACKET_SIZE, sizeof(UDPFirstPacket) + shapeShifter.size());
	UDPFirstPacket* first = (UDPFirstPacket*)buf;

	uint16_t msg_id = m_sender->allocateMessageID();

	first->header.frag_id = 0;
	first->header.msg_id = msg_id;

	// Calculate number of packets
	first->header.remaining_packets = std::max<uint32_t>(0,
		(shapeShifter.size() - UDPFirstPacket::MaxDataSize + (UDPDataPacket::MaxDataSize-1)) / UDPDataPacket::MaxDataSize
	);

	strncpy(first->header.topic_name, m_topicName.c_str(), sizeof(first->header.topic_name));
	if(first->header.topic_name[sizeof(first->header.topic_name)-1] != 0)
	{
		ROS_ERROR("Topic '%s' is too long. Please shorten the name.", m_topicName.c_str());
		first->header.topic_name[sizeof(first->header.topic_name)-1] = 0;
	}

	strncpy(first->header.topic_type, shapeShifter.getDataType().c_str(), sizeof(first->header.topic_type));
	if(first->header.topic_type[sizeof(first->header.topic_type)-1] != 0)
	{
		ROS_ERROR("Topic type '%s' is too long. Please shorten the name.", shapeShifter.getDataType().c_str());
		first->header.topic_type[sizeof(first->header.topic_type)-1] = 0;
	}

	std::string md5 = shapeShifter.getMD5Sum();
	for(int i = 0; i < 4; ++i)
	{
		std::string md5_part = md5.substr(4*i, 4);
		uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
		first->header.topic_md5[i] = md5_num;
	}

	uint32_t size = shapeShifter.size();
	if(size > m_buf.size())
		m_buf.resize(size);
	shapeShifter.write(*this);

	uint8_t* rptr = m_buf.data();
	uint32_t psize = std::min<uint32_t>(UDPFirstPacket::MaxDataSize, size);
	memcpy(first->data, rptr, psize);
	rptr += psize;
	size -= psize;

	if(!m_sender->send(buf, buf_size))
		return;

	uint16_t frag_id = 1;
	while(size > 0)
	{
		buf_size = std::min<uint32_t>(PACKET_SIZE, sizeof(UDPDataPacket) + size);
		UDPDataPacket* next_packet = (UDPDataPacket*)buf;
		next_packet->header.frag_id = frag_id++;
		next_packet->header.msg_id = msg_id;

		psize = std::min<uint32_t>(UDPDataPacket::MaxDataSize, size);
		memcpy(next_packet->data, rptr, psize);
		rptr += psize;
		size -= psize;

		if(!m_sender->send(buf, buf_size))
			return;
	}
}



}
