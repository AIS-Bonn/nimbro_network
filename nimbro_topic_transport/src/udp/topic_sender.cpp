// Sends a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_sender.h"
#include "udp_sender.h"
#include "udp_packet.h"

#include <bzlib.h>

#include <nimbro_topic_transport/CompressedMsg.h>

#include <boost/algorithm/string/replace.hpp>

#if WITH_RAPTORQ
#include <RaptorQ/RaptorQ.hpp>
#endif

namespace nimbro_topic_transport
{

TopicSender::TopicSender(UDPSender* sender, ros::NodeHandle* nh, const std::string& topic, double rate, bool resend, int flags, bool enable)
 : m_sender(sender)
 , m_flags(flags)
 , m_updateBuf(true)
 , m_msgCounter(0)
 , m_inputMsgCounter(0)
 , m_directTransmission(true)
 , m_enable(escapeTopicName(topic), enable)
{
	ROS_INFO_STREAM("Subscribing to" << topic);
	m_subscriber = nh->subscribe(topic, 1, &TopicSender::handleData, this);
	m_topicName = topic;

	m_durationBetweenPackets = ros::Duration(1.0 / rate);

	if(resend)
	{
		m_resendTimer = nh->createTimer(m_durationBetweenPackets, boost::bind(&TopicSender::resend, this));
		m_resendTimer.start();
	}
}

TopicSender::~TopicSender()
{
	ROS_DEBUG("Topic '%s': Sent %d messages", m_topicName.c_str(), m_msgCounter);
}

void TopicSender::send()
{
	if(m_updateBuf)
	{
		boost::lock_guard<boost::mutex> lock(m_dataMutex);

		if(!m_lastData)
			return;

		// If the data was sent over our CompressedMsg format, do not recompress it
		if((m_flags & UDP_FLAG_COMPRESSED) && m_lastData->getDataType() == "nimbro_topic_transport/CompressedMsg")
		{
			CompressedMsg::Ptr compressed = m_lastData->instantiate<CompressedMsg>();
			if(!compressed)
			{
				ROS_ERROR("Could not instantiate CompressedMsg");
				return;
			}

			m_buf.swap(compressed->data);
			memcpy(m_md5, compressed->md5.data(), sizeof(m_md5));
			m_topicType = compressed->type;
		}
		else
		{
			m_buf.resize(m_lastData->size());
			m_lastData->write(*this);

			if(m_flags & UDP_FLAG_COMPRESSED)
			{
				unsigned int len = m_buf.size() + m_buf.size() / 100 + 1200;
				m_compressionBuf.resize(len);
				int ret = BZ2_bzBuffToBuffCompress((char*)m_compressionBuf.data(), &len, (char*)m_buf.data(), m_buf.size(), 3, 0, 30);
				if(ret == BZ_OK)
				{
					m_buf.swap(m_compressionBuf);
					m_buf.resize(len);
				}
				else
				{
					ROS_ERROR("Could not compress data, sending uncompressed");
				}
			}

			std::string md5 = m_lastData->getMD5Sum();
			for(int i = 0; i < 4; ++i)
			{
				std::string md5_part = md5.substr(8*i, 8);
				uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
				m_md5[i] = md5_num;
			}

			m_topicType = m_lastData->getDataType();
		}

		m_updateBuf = false;
	}

	// Do we want to do forward error correction?
	if(m_sender->fec() != 0.0)
	{
		sendWithFEC();
	}
	else
	{
		sendWithoutFEC();
	}

	m_msgCounter++;
}

void TopicSender::sendWithFEC()
{
#if WITH_RAPTORQ
	// Prepend a FECHeader (FIXME: Can we avoid the data copy?)
	std::vector<uint8_t> input(sizeof(FECHeader) + m_buf.size());
	FECHeader* header = reinterpret_cast<FECHeader*>(input.data());
	memcpy(input.data() + sizeof(FECHeader), m_buf.data(), m_buf.size());

	// Fill in header fields
	header->flags = m_flags;
	header->topic_msg_counter = m_inputMsgCounter;

	strncpy(header->topic_name, m_topicName.c_str(), sizeof(header->topic_name));
	if(header->topic_name[sizeof(header->topic_name)-1] != 0)
	{
		ROS_ERROR("Topic '%s' is too long. Please shorten the name.", m_topicName.c_str());
		header->topic_name[sizeof(header->topic_name)-1] = 0;
	}

	strncpy(header->topic_type, m_topicType.c_str(), sizeof(header->topic_type));
	if(header->topic_type[sizeof(header->topic_type)-1] != 0)
	{
		ROS_ERROR("Topic type '%s' is too long. Please shorten the name.", m_topicType.c_str());
		header->topic_type[sizeof(header->topic_type)-1] = 0;
	}

	for(int i = 0; i < 4; ++i)
		header->topic_md5[i] = m_md5[i];

	using IT = typename std::vector<uint8_t>::iterator;
	RaptorQ::Encoder<IT, IT> encoder(input.begin(), input.end(), 4, FECPacket::MaxDataSize, 50000);

	if(!encoder)
		throw std::runtime_error("Could not initialize encoder");

	ros::WallTime start = ros::WallTime::now();
	encoder.precompute(6, false);
	ros::WallTime end = ros::WallTime::now();
	ROS_INFO("FEC took %f ms and produced %d blocks", (end - start).toSec() * 1000, encoder.blocks());

	std::vector<uint8_t> output(PACKET_SIZE);
	FECPacket* outPacket = reinterpret_cast<FECPacket*>(output.data());

	outPacket->header.msg_id = m_sender->allocateMessageID();
	outPacket->header.oti_common = encoder.OTI_Common();
	outPacket->header.oti_specific = encoder.OTI_Scheme_Specific();

	unsigned int packets = 0;

	for(const auto& block : encoder)
	{
		// First send source symbols
		for(auto it = block.begin_source(); it != block.end_source(); ++it)
		{
			outPacket->header.symbol_id = (*it).id();

			auto dataStart = output.begin() + sizeof(FECPacket::Header);
			uint64_t dataSize = (*it)(dataStart, output.end());

			if(!m_sender->send(output.data(), sizeof(FECPacket::Header) + dataSize))
				return;
			packets++;
		}

		// Then repair symbols
		unsigned int sentRepair = 0;
		const unsigned int numRepair = std::ceil(m_sender->fec() * block.symbols());

		for(auto it = block.begin_repair(); it != block.end_repair(block.max_repair()) && sentRepair < numRepair; ++it)
		{
			outPacket->header.symbol_id = (*it).id();

			auto dataStart = output.begin() + sizeof(FECPacket::Header);
			uint64_t dataSize = (*it)(dataStart, output.end());

			if(!m_sender->send(output.data(), sizeof(FECPacket::Header) + dataSize))
				return;

			sentRepair++;
			packets++;
		}
	}

	ROS_INFO("FEC: Sent msg id %u with %u packets", outPacket->header.msg_id(), packets);

#else
	throw std::runtime_error("Forward error correction requested, but I was not compiled with RaptorQ support...");
#endif
}

void TopicSender::sendWithoutFEC()
{
	uint32_t size = m_buf.size();

	uint8_t buf[PACKET_SIZE];
	uint32_t buf_size = std::min<uint32_t>(PACKET_SIZE, sizeof(UDPFirstPacket) + size);
	UDPFirstPacket* first = (UDPFirstPacket*)buf;

	uint16_t msg_id = m_sender->allocateMessageID();

	first->header.frag_id = 0;
	first->header.msg_id = msg_id;
	first->header.flags = m_flags;
	first->header.topic_msg_counter = m_inputMsgCounter;

	// Calculate number of packets
	first->header.remaining_packets = std::max<uint32_t>(0,
		(size - UDPFirstPacket::MaxDataSize + (UDPDataPacket::MaxDataSize-1)) / UDPDataPacket::MaxDataSize
	);

	strncpy(first->header.topic_name, m_topicName.c_str(), sizeof(first->header.topic_name));
	if(first->header.topic_name[sizeof(first->header.topic_name)-1] != 0)
	{
		ROS_ERROR("Topic '%s' is too long. Please shorten the name.", m_topicName.c_str());
		first->header.topic_name[sizeof(first->header.topic_name)-1] = 0;
	}

	strncpy(first->header.topic_type, m_topicType.c_str(), sizeof(first->header.topic_type));
	if(first->header.topic_type[sizeof(first->header.topic_type)-1] != 0)
	{
		ROS_ERROR("Topic type '%s' is too long. Please shorten the name.", m_topicType.c_str());
		first->header.topic_type[sizeof(first->header.topic_type)-1] = 0;
	}

	for(int i = 0; i < 4; ++i)
		first->header.topic_md5[i] = m_md5[i];

	uint8_t* rptr = m_buf.data();
	uint32_t psize = std::min<uint32_t>(UDPFirstPacket::MaxDataSize, size);
	memcpy(first->data, rptr, psize);
	rptr += psize;
	size -= psize;

	if(!m_sender->send(buf, buf_size))
		return;

	if(m_sender->duplicateFirstPacket())
	{
		if(!m_sender->send(buf, buf_size))
			return;
	}

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

void TopicSender::handleData(const topic_tools::ShapeShifter::ConstPtr& shapeShifter)
{
	if (!m_enable() ) 
		return; 
	
	{
		boost::lock_guard<boost::mutex> lock(m_dataMutex);

		m_lastData = shapeShifter;
		m_updateBuf = true;

		ros::Time now = ros::Time::now();
		if(now - m_lastTime < m_durationBetweenPackets)
			return;

		m_lastTime = now;
		m_inputMsgCounter++;
	}

	if(m_directTransmission)
		send();
}

void TopicSender::resend()
{
	if(!m_lastData)
		return;

	ros::Time now = ros::Time::now();
	if(now - m_lastTime < m_durationBetweenPackets)
		return;

	sendCurrentMessage();
}

void TopicSender::sendCurrentMessage()
{
	if(!m_lastData)
		return;

	send();
}

void TopicSender::setDirectTransmissionEnabled(bool value)
{
	m_directTransmission = value;
	if(m_directTransmission && m_resendTimer.isValid())
		m_resendTimer.start();
	else
		m_resendTimer.stop();
}


std::string TopicSender::escapeTopicName(std::string topicName)
{
	boost::replace_first(topicName, "/", "");
	boost::replace_all(topicName, "/", "_");
	return topicName;
}

}
