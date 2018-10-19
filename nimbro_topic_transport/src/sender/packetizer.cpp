// Fragment a message into (UDP) packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "packetizer.h"

#include <random>

#include <wirehair/wirehair.h>

#include "../udp_packet.h"

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

	if(wirehair_init() != Wirehair_Success)
		throw std::runtime_error("Could not initialize wirehair FEC library");
}

TopicPacketizer::~TopicPacketizer()
{
}

std::vector<Packet::Ptr> TopicPacketizer::packetize(const Message::ConstPtr& msg)
{
	// Acquire a unique message ID
	uint16_t messageID = m_packetizer->allocateMessageID();

	uint64_t dataSize = sizeof(UDPData::Header) + msg->payload.size();

	// Packet generation
	uint64_t sourceSymbols = div_round_up(dataSize, UDPPacket::MaxDataSize);
	uint64_t repairSymbols = std::ceil(m_fec * sourceSymbols);

	uint64_t paddedSize = sourceSymbols * UDPPacket::MaxDataSize;

	std::vector<uint8_t> messageData;
	if(repairSymbols != 0 && sourceSymbols > 1)
		messageData.resize(paddedSize);
	else
		messageData.resize(dataSize);

	UDPData* data = reinterpret_cast<UDPData*>(messageData.data());

	// Fill in UDPData header
	{
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
	}

	// Copy data
	memcpy(messageData.data() + sizeof(UDPData::Header), msg->payload.data(), msg->payload.size());

	uint32_t seed = rand(); // TODO: REMOVE

	std::vector<Packet::Ptr> packets;

	// Write source packets
	{
		uint64_t remaining = messageData.size();
		const uint8_t* readPtr = messageData.data();

		for(unsigned int packetID = 0; remaining != 0; ++packetID)
		{
			uint64_t take_now = std::min<uint64_t>(UDPPacket::MaxDataSize, remaining);
			auto packetBuf = std::make_shared<Packet>();

			if(repairSymbols != 0 && sourceSymbols > 1 && take_now != UDPPacket::MaxDataSize)
			{
				throw std::logic_error("unaligned");
			}

			UDPPacket* packet = packetBuf->packet();

			packet->header.msg_id = messageID;
			packet->header.source_symbols = sourceSymbols;
			packet->header.repair_symbols = repairSymbols;
			packet->header.symbol_id = packetID;
			packet->header.prng_seed = seed;

			packetBuf->length = sizeof(UDPPacket::Header) + take_now;

			memcpy(packet->data, readPtr, take_now);
			readPtr += take_now;
			remaining -= take_now;

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
		if(sourceSymbols < 2)
		{
			// Just repeat the symbol
			for(uint64_t i = 0; i < repairSymbols; ++i)
			{
				auto repeatedBuf = std::make_shared<Packet>(*packets[0]);
				UDPPacket* packet = repeatedBuf->packet();

				packet->header.symbol_id = 1 + i;

				packets.emplace_back(std::move(repeatedBuf));
			}
		}
		else
		{
			WirehairCodec encoder = wirehair_encoder_create(nullptr, messageData.data(), messageData.size(), UDPPacket::MaxDataSize);
			if(!encoder)
			{
				ROS_ERROR("Could not create wirehair encoder!");
				return {};
			}

			for(std::size_t i = 0; i < repairSymbols; ++i)
			{
				auto buf = std::make_shared<Packet>();
				UDPPacket* packet = buf->packet();

				packet->header.msg_id = messageID;
				packet->header.source_symbols = sourceSymbols;
				packet->header.repair_symbols = repairSymbols;
				packet->header.symbol_id = sourceSymbols + i;
				packet->header.prng_seed = seed;

				uint32_t length = 0;

				WirehairResult res = wirehair_encode(encoder, sourceSymbols + i, packet->data, UDPPacket::MaxDataSize, &length);
				if(res != Wirehair_Success)
				{
					ROS_ERROR("Wirehair encode() error: %s", wirehair_result_string(res));
					return {};
				}

				if(length != UDPPacket::MaxDataSize)
				{
					ROS_ERROR("Unexpected wirehair output size: %u, expected %u",
						length, UDPPacket::MaxDataSize
					);
				}

				buf->length = length + sizeof(UDPPacket::Header);

				packets.emplace_back(std::move(buf));
			}

			wirehair_free(encoder);
		}

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
