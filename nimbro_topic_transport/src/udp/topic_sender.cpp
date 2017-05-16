// Sends a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_sender.h"
#include "udp_sender.h"
#include "udp_packet.h"
#include "../topic_info.h"

#include <bzlib.h>

#include <nimbro_topic_transport/CompressedMsg.h>

#include <boost/algorithm/string/replace.hpp>

#include <random>
#include <algorithm>
#include <chrono>

#if WITH_OPENFEC
extern "C"
{
#include <of_openfec_api.h>
}
#endif

#include <fcntl.h>

namespace nimbro_topic_transport
{

TopicSender::TopicSender(UDPSender* sender, ros::NodeHandle* nh, const std::string& topic, double rate, bool resend, int flags, bool enable, const std::string& type)
 : m_sender(sender)
 , m_flags(flags)
 , m_updateBuf(true)
 , m_msgCounter(0)
 , m_inputMsgCounter(0)
 , m_directTransmission(true)
#if WITH_CONFIG_SERVER
 , m_enable(escapeTopicName(topic), enable)
#endif
{
	ros::SubscribeOptions ops;
	boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> func = boost::bind(&TopicSender::handleData, this, _1);
	ops.initByFullCallbackType(topic, 20, func);

	if(!type.empty())
	{
		ops.datatype = type;
		ops.md5sum = topic_info::getMd5Sum(type);
	}

	m_subscriber = nh->subscribe(ops);
	m_topicName = topic;

	if(rate == 0.0)
		m_durationBetweenPackets = ros::Duration(0.0);
	else
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
				uint32_t md5_num = strtoll(md5_part.c_str(), 0, 16);
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

inline uint64_t div_round_up(uint64_t a, uint64_t b)
{
	return (a + b - 1) / b;
}

void TopicSender::sendWithFEC()
{
#if WITH_OPENFEC
	uint16_t msg_id = m_sender->allocateMessageID();
	uint64_t dataSize = sizeof(FECHeader) + m_buf.size();

	// If the message fits in a single packet, use that as the buffer size
	uint64_t symbolSize;
	uint64_t sourceSymbols;

	if(dataSize <= FECPacket::MaxDataSize)
	{
		sourceSymbols = 1;
		symbolSize = dataSize;
	}
	else
	{
		// We need to pad the data to a multiple of our packet payload size.
		sourceSymbols = div_round_up(dataSize, FECPacket::MaxDataSize);
		symbolSize = FECPacket::MaxDataSize;
	}

	ROS_DEBUG("dataSize: %lu, symbol size: %lu, sourceSymbols: %lu", dataSize, symbolSize, sourceSymbols);

	uint64_t packetSize = sizeof(FECPacket::Header) + symbolSize;

	ROS_DEBUG("=> packetSize: %lu", packetSize);

	uint64_t repairSymbols = std::ceil(m_sender->fec() * sourceSymbols);

	uint64_t numPackets = sourceSymbols + repairSymbols;

	of_session_t* ses = 0;
	uint32_t prng_seed = rand();
	if(sourceSymbols >= MIN_PACKETS_LDPC)
	{
		ROS_DEBUG("%s: Choosing LDPC-Staircase codec", m_topicName.c_str());

		if(of_create_codec_instance(&ses, OF_CODEC_LDPC_STAIRCASE_STABLE, OF_ENCODER, 1) != OF_STATUS_OK)
		{
			ROS_ERROR("%s: Could not create LDPC codec instance", m_topicName.c_str());
			return;
		}

		of_ldpc_parameters_t params;
		params.nb_source_symbols = sourceSymbols;
		params.nb_repair_symbols = std::ceil(m_sender->fec() * sourceSymbols);
		params.encoding_symbol_length = symbolSize;
		params.prng_seed = prng_seed;
		params.N1 = 7;

		ROS_DEBUG("LDPC seed: 7, 0x%X", params.prng_seed);

		if(of_set_fec_parameters(ses, (of_parameters_t*)&params) != OF_STATUS_OK)
		{
			ROS_ERROR("%s: Could not set FEC parameters", m_topicName.c_str());
			of_release_codec_instance(ses);
			return;
		}
	}
	else
	{
		ROS_DEBUG("%s: Choosing Reed-Solomon codec", m_topicName.c_str());

		if(of_create_codec_instance(&ses, OF_CODEC_REED_SOLOMON_GF_2_M_STABLE, OF_ENCODER, 0) != OF_STATUS_OK)
		{
			ROS_ERROR("%s: Could not create REED_SOLOMON codec instance", m_topicName.c_str());
			return;
		}

		of_rs_2_m_parameters params;
		params.nb_source_symbols = sourceSymbols;
		params.nb_repair_symbols = std::ceil(m_sender->fec() * sourceSymbols);
		params.encoding_symbol_length = symbolSize;
		params.m = 8;

		if(of_set_fec_parameters(ses, (of_parameters_t*)&params) != OF_STATUS_OK)
		{
			ROS_ERROR("%s: Could not set FEC parameters", m_topicName.c_str());
			of_release_codec_instance(ses);
			return;
		}
	}

	std::vector<uint8_t> packetBuffer(numPackets * packetSize);
	std::vector<void*> symbols(sourceSymbols + repairSymbols);

	uint64_t writtenData = 0;

	// Fill the source packets
	for(uint64_t i = 0; i < sourceSymbols; ++i)
	{
		uint8_t* packetPtr = packetBuffer.data() + i * packetSize;

		FECPacket::Header* header = reinterpret_cast<FECPacket::Header*>(packetPtr);

		header->msg_id = msg_id;
		header->symbol_id = i;
		header->symbol_length = symbolSize;
		header->source_symbols = sourceSymbols;
		header->repair_symbols = repairSymbols;
		header->prng_seed = prng_seed;

		uint8_t* dataPtr = packetPtr + sizeof(FECPacket::Header);
		uint64_t remainingSpace = symbolSize;

		symbols[i] = dataPtr;

		if(i == 0)
		{
			// First packet includes the FECHeader
			FECHeader* msgHeader = reinterpret_cast<FECHeader*>(dataPtr);

			// Fill in header fields
			msgHeader->flags = m_flags;
			msgHeader->topic_msg_counter = m_inputMsgCounter;

			strncpy(msgHeader->topic_name, m_topicName.c_str(), sizeof(msgHeader->topic_name));
			if(msgHeader->topic_name[sizeof(msgHeader->topic_name)-1] != 0)
			{
				ROS_ERROR("Topic '%s' is too long. Please shorten the name.", m_topicName.c_str());
				msgHeader->topic_name[sizeof(msgHeader->topic_name)-1] = 0;
			}

			strncpy(msgHeader->topic_type, m_topicType.c_str(), sizeof(msgHeader->topic_type));
			if(msgHeader->topic_type[sizeof(msgHeader->topic_type)-1] != 0)
			{
				ROS_ERROR("Topic type '%s' is too long. Please shorten the name.", m_topicType.c_str());
				msgHeader->topic_type[sizeof(msgHeader->topic_type)-1] = 0;
			}

			for(int i = 0; i < 4; ++i)
				msgHeader->topic_md5[i] = m_md5[i];

			dataPtr += sizeof(FECHeader);
			remainingSpace -= sizeof(FECHeader);
		}

		uint64_t chunkSize = std::min(remainingSpace, m_buf.size() - writtenData);
		memcpy(dataPtr, m_buf.data() + writtenData, chunkSize);
		writtenData += chunkSize;

		// Set any padding to zero
		if(chunkSize < remainingSpace)
			memset(dataPtr + chunkSize, 0, remainingSpace - chunkSize);
	}

	// Fill the repair packets
	for(uint64_t i = sourceSymbols; i < sourceSymbols + repairSymbols; ++i)
	{
		uint8_t* packetPtr = packetBuffer.data() + i * packetSize;

		FECPacket::Header* header = reinterpret_cast<FECPacket::Header*>(packetPtr);

		header->msg_id = msg_id;
		header->symbol_id = i;
		header->symbol_length = symbolSize;
		header->source_symbols = sourceSymbols;
		header->repair_symbols = repairSymbols;
		header->prng_seed = prng_seed;

		uint8_t* dataPtr = packetPtr + sizeof(FECPacket::Header);
		symbols[i] = dataPtr;
	}
	for(uint64_t i = sourceSymbols; i < sourceSymbols + repairSymbols; ++i)
	{
		if(of_build_repair_symbol(ses, symbols.data(), i) != OF_STATUS_OK)
		{
			ROS_ERROR("%s: Could not build repair symbol", m_topicName.c_str());
			of_release_codec_instance(ses);
			return;
		}
	}

	// FEC work is done
	of_release_codec_instance(ses);

	std::vector<unsigned int> packetOrder(numPackets);
	std::iota(packetOrder.begin(), packetOrder.end(), 0);

	// Send the packets in random order
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 mt(seed);
	std::shuffle(packetOrder.begin(), packetOrder.end(), mt);

	ROS_DEBUG("Sending %d packets", (int)packetOrder.size());
	for(unsigned int idx : packetOrder)
	{
		if(!m_sender->send(packetBuffer.data() + idx * packetSize, packetSize, m_topicName))
			return;
	}
#else
	throw std::runtime_error("Forward error correction requested, but I was not compiled with FEC support...");
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

	if(!m_sender->send(buf, buf_size, m_topicName))
		return;

	if(m_sender->duplicateFirstPacket())
	{
		if(!m_sender->send(buf, buf_size, m_topicName))
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

		if(!m_sender->send(buf, buf_size, m_topicName))
			return;
	}
}

void TopicSender::handleData(const topic_tools::ShapeShifter::ConstPtr& shapeShifter)
{
#if WITH_CONFIG_SERVER
	if (!m_enable() ) 
		return;
#endif

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
