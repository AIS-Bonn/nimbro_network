// Fragment a message into (UDP) packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "packetizer.h"

#include "../udp_packet.h"

extern "C"
{
#include <of_openfec_api.h>
}

namespace nimbro_topic_transport
{

namespace
{
	inline uint64_t div_round_up(uint64_t a, uint64_t b)
	{
		return (a + b - 1) / b;
	}
}

uint32_t Packetizer::allocateMessageID()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_messageID++;
}

TopicPacketizer::TopicPacketizer(const Packetizer::Ptr& packetizer, const Topic::ConstPtr& topic)
 : m_packetizer(packetizer)
{
	auto config = topic->config;
	if(config.hasMember("fec"))
	{
		m_fec = (double)config["fec"];
		if(m_fec < 0)
			throw std::runtime_error("Invalid FEC percentage.");
	}
}

TopicPacketizer::~TopicPacketizer()
{
}

std::vector<Packet::Ptr> TopicPacketizer::packetize(const Message::ConstPtr& msg)
{
	// Acquire a unique message ID
	uint16_t messageID = m_packetizer->allocateMessageID();

	uint64_t dataSize = sizeof(UDPData::Header) + msg->payload.size();
	uint64_t sourceSymbols = div_round_up(dataSize, UDPPacket::MaxDataSize);
	uint64_t repairSymbols = std::ceil(m_fec * sourceSymbols);
	uint64_t symbolSize = std::min<uint64_t>(UDPPacket::MaxDataSize, dataSize);
	uint16_t seed = rand() & 0xFFFF;

	std::vector<Packet::Ptr> packets;

	// Write source packets
	{
		uint64_t remaining = dataSize;
		const uint8_t* readPtr = msg->payload.data();

		for(unsigned int packetID = 0; remaining != 0; ++packetID)
		{
			uint64_t take_now = std::min<uint64_t>(UDPPacket::MaxDataSize, remaining);
			auto packetBuf = std::make_shared<Packet>();

			UDPPacket* packet = packetBuf->packet();

			packet->header.msg_id = messageID;
			packet->header.source_symbols = sourceSymbols;
			packet->header.repair_symbols = repairSymbols;
			packet->header.symbol_id = packetID;
			packet->header.prng_seed = seed;

			packetBuf->length = sizeof(UDPPacket::Header) + take_now;

			if(packetID == 0)
			{
				// First packet includes the data header
				UDPData* data = reinterpret_cast<UDPData*>(packet->data);

				data->header.flags = msg->flags;
				data->header.topic_msg_counter = msg->counter;

				strncpy(data->header.topic_name, msg->topic->name.c_str(), sizeof(data->header.topic_name));
				if(data->header.topic_name[sizeof(data->header.topic_name)-1] != 0)
				{
					ROS_ERROR_THROTTLE(0.5,
						"Topic '%s' is too long. Please shorten the name.",
						msg->topic->name.c_str()
					);
					data->header.topic_name[sizeof(data->header.topic_name)-1] = 0;
				}

				strncpy(data->header.topic_type, msg->type.c_str(), sizeof(data->header.topic_type));
				if(data->header.topic_type[sizeof(data->header.topic_type)-1] != 0)
				{
					ROS_ERROR_THROTTLE(0.5,
						"Topic type '%s' is too long. Please shorten the name.",
						msg->topic->name.c_str()
					);
					data->header.topic_type[sizeof(data->header.topic_type)-1] = 0;
				}

				for(int i = 0; i < 4; ++i)
				{
					std::string md5_part = msg->md5.substr(8*i, 8);
					uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
					data->header.topic_md5[i] = md5_num;
				}

				data->header.size = msg->payload.size();

				// Fill remainder with data
				take_now -= sizeof(UDPData::Header);
				remaining -= sizeof(UDPData::Header);

				memcpy(data->data, readPtr, take_now);
				readPtr += take_now;
				remaining -= take_now;
			}
			else
			{
				// Just data
				memcpy(packet->data, readPtr, take_now);
				readPtr += take_now;
				remaining -= take_now;
			}

			// Fill any leftover regions (padding for FEC) with zeroes
			memset(packetBuf->data.data() + packetBuf->length, 0,
				packetBuf->data.size() - packetBuf->length
			);

			packets.emplace_back(std::move(packetBuf));
		}
	}

	// Create repair symbols
	if(repairSymbols != 0)
	{
		of_session_t* ses = 0;

		if(sourceSymbols >= MIN_PACKETS_LDPC)
		{
			ROS_DEBUG("%s: Choosing LDPC-Staircase codec", msg->topic->name.c_str());

			if(of_create_codec_instance(&ses, OF_CODEC_LDPC_STAIRCASE_STABLE, OF_ENCODER, 1) != OF_STATUS_OK)
			{
				ROS_ERROR("%s: Could not create LDPC codec instance, sending without FEC", msg->topic->name.c_str());
				return packets;
			}

			of_ldpc_parameters_t params;
			params.nb_source_symbols = sourceSymbols;
			params.nb_repair_symbols = repairSymbols;
			params.encoding_symbol_length = symbolSize;
			params.prng_seed = seed;
			params.N1 = 7;

			if(of_set_fec_parameters(ses, (of_parameters_t*)&params) != OF_STATUS_OK)
			{
				ROS_ERROR("%s: Could not set FEC parameters, sending without FEC", msg->topic->name.c_str());
				of_release_codec_instance(ses);
				return packets;
			}
		}
		else
		{
			ROS_DEBUG("%s: Choosing Reed-Solomon codec", msg->topic->name.c_str());

			if(of_create_codec_instance(&ses, OF_CODEC_REED_SOLOMON_GF_2_M_STABLE, OF_ENCODER, 0) != OF_STATUS_OK)
			{
				ROS_ERROR("%s: Could not create REED_SOLOMON codec instance, sending without FEC", msg->topic->name.c_str());
				return packets;
			}

			of_rs_2_m_parameters params;
			params.nb_source_symbols = sourceSymbols;
			params.nb_repair_symbols = repairSymbols;
			params.encoding_symbol_length = symbolSize;
			params.m = 8;

			if(of_set_fec_parameters(ses, (of_parameters_t*)&params) != OF_STATUS_OK)
			{
				ROS_ERROR("%s: Could not set FEC parameters, sending without FEC", msg->topic->name.c_str());
				of_release_codec_instance(ses);
				return packets;
			}
		}

		for(uint64_t i = 0; i < repairSymbols; ++i)
		{
			auto packetBuf = std::make_shared<Packet>();

			UDPPacket* packet = packetBuf->packet();

			packet->header.msg_id = messageID;
			packet->header.source_symbols = sourceSymbols;
			packet->header.repair_symbols = repairSymbols;
			packet->header.symbol_id = sourceSymbols + i;
			packet->header.prng_seed = seed;

			packetBuf->length = sizeof(UDPPacket::Header) + symbolSize;

			packets.emplace_back(std::move(packetBuf));
		}

		std::vector<void*> fecSymbols;
		fecSymbols.reserve(packets.size());
		for(auto& packet : packets)
			fecSymbols.push_back(packet->packet()->data);

		for(uint64_t i = sourceSymbols; i < sourceSymbols + repairSymbols; ++i)
		{
			if(of_build_repair_symbol(ses, fecSymbols.data(), i) != OF_STATUS_OK)
			{
				ROS_ERROR("%s: Could not build repair symbol", msg->topic->name.c_str());
				of_release_codec_instance(ses);
				packets.resize(sourceSymbols);
				return packets;
			}
		}

		// FEC work is done
		of_release_codec_instance(ses);

		{
			// Send the packets in random order
			unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			std::mt19937 mt(seed);
			std::shuffle(packets.begin(), packets.end(), mt);
		}
	}

	return packets;
}

}
