// Fragment a message into (UDP) packets
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "packetizer.h"

#include "../udp_packet.h"

#include <of_openfec_api.h>

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

std::vector<Packet> TopicPacketizer::packetize(const Message::ConstPtr& msg)
{
	// Acquire a unique message ID
	uint16_t messageID = m_packetizer->allocateMessageID();

	uint64_t dataSize = sizeof(UDPData::Header) + msg->payload.size();
	uint64_t sourceSymbols = div_round_up(dataSize, UDPPacket::MaxDataSize);
	uint64_t repairSymbols = std::ceil(m_fec * sourceSymbols);

	std::vector<Packet> packets;
	std::vector<void*> fecSymbols;

	// Write source packets
	{
		uint64_t remaining = dataSize;
		const uint8_t* readPtr = msg->payload.data();

		for(unsigned int packetID = 0; remaining != 0; ++packetID)
		{
			uint64_t take_now = std::min<uint64_t>(UDPPacket::MaxDataSize, remaining);
			Packet packetBuf;
			packetBuf.data.resize(PACKET_SIZE);

			UDPPacket* packet = reinterpret_cast<UDPPacket*>(packetBuf.data.data());

			packet->header.msg_id = messageID;
			packet->header.source_symbols = sourceSymbols;
			packet->header.repair_symbols = repairSymbols;
			packet->header.symbol_id = packetID;

			packetBuf.length = sizeof(UDPPacket::Header) + take_now;

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
				memcpy(data->data, readPtr, take_now);
				readPtr += take_now;
			}
			else
			{
				// Just data
				memcpy(packet->data, readPtr, take_now);
				readPtr += take_now;
			}

			// Fill any leftover regions (padding for FEC) with zeroes
			memset(packetBuf.data.data() + packetBuf.length, 0,
				packetBuf.data.size() - packetBuf.length
			);

			packets.emplace_back(std::move(packetBuf));
			fecSymbols.push_back(packets.back().data.data());
		}
	}

	// Create repair symbols
	if(repairSymbols != 0)
	{
		of_session_t* ses = 0;
	}

	return packets;
}

}
