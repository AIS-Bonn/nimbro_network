// De-fragments packets into complete messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "depacketizer.h"

#include <algorithm>

#include <ros/console.h>

#include <wirehair/wirehair.h>

#include "../topic_info.h"

namespace nimbro_topic_transport
{

Depacketizer::Depacketizer()
{
}

void Depacketizer::setCallback(const Callback& cb)
{
	m_cb = cb;
}

void Depacketizer::addPacket(const Packet::Ptr& packet)
{
	std::unique_lock<std::mutex> lock(m_mutex);

	uint32_t msg_id = packet->packet()->header.msg_id;

	auto it = std::find_if(m_messageBuffer.begin(), m_messageBuffer.end(),
		[=](const PartialMessage& m) { return m.id == msg_id; }
	);

	if(it == m_messageBuffer.end())
	{
		m_messageBuffer.push_front(PartialMessage(msg_id));
		it = m_messageBuffer.begin();

		pruneMessages();
	}

	handleMessagePacket(it, packet);
}

void Depacketizer::handleMessagePacket(std::list<PartialMessage>::iterator it, const Packet::Ptr& packet)
{
	PartialMessage* msg = &*it;

	if(msg->complete)
		return;

	const auto& header = packet->packet()->header;

	if(header.source_symbols() == 0)
	{
		ROS_WARN_THROTTLE(1.0, "Don't know how to handle source_symbols == 0, dropping packet. This message is throttled.");
		return;
	}

	uint64_t totalSymbols = header.source_symbols() + header.repair_symbols();
	msg->packets.resize(totalSymbols);

	msg->packets[header.symbol_id()] = packet;

	std::vector<uint8_t> recoveredData;

	// Do we need to do FEC decoding?
	if(header.repair_symbols())
	{
		if(header.source_symbols() > 1)
		{
			if(!msg->decoder)
			{
				msg->decoder.reset(wirehair_decoder_create(nullptr, header.source_symbols() * UDPPacket::MaxDataSize, UDPPacket::MaxDataSize), wirehair_free);
			}

			WirehairResult res = wirehair_decode(msg->decoder.get(), header.symbol_id, packet->packet()->data, packet->length - sizeof(UDPPacket::Header));
			if(res == Wirehair_NeedMore)
				return;

			if(res != Wirehair_Success)
			{
				ROS_ERROR("Wirehair decode() error: %s", wirehair_result_string(res));
				ROS_ERROR(" ^- on packet with symbol_id=%u, length=%lu", header.symbol_id(), packet->length);
				ROS_ERROR(" ^- source_symbols=%u, repair_symbols=%u", header.source_symbols(), header.repair_symbols());
				return;
			}

			recoveredData.resize(header.source_symbols() * UDPPacket::MaxDataSize);
			res = wirehair_recover(msg->decoder.get(), recoveredData.data(), recoveredData.size());
			if(res != Wirehair_Success)
			{
				ROS_ERROR("Wirehair recover() error: %s", wirehair_result_string(res));
				return;
			}
		}
		else
		{
			recoveredData.resize(packet->length - sizeof(UDPPacket::Header));
			memcpy(recoveredData.data(), packet->packet()->data, packet->length - sizeof(UDPPacket::Header));
		}
	}
	else
	{
		msg->received_symbols++;
		if(msg->received_symbols < header.source_symbols())
			return;

		if(header.source_symbols() == 1)
		{
			recoveredData.resize(packet->length);
			memcpy(recoveredData.data(), packet->packet()->data, packet->length - sizeof(UDPPacket::Header));
		}
		else
		{
			recoveredData.resize(header.source_symbols() * UDPPacket::MaxDataSize);

			std::size_t off = 0;
			for(std::size_t i = 0; i < header.source_symbols(); ++i)
			{
				std::size_t len = msg->packets[i]->length - sizeof(UDPPacket::Header);
				memcpy(
					recoveredData.data() + off,
					msg->packets[i]->packet()->data,
					len
				);
				off += len;
			}
		}
	}

	msg->complete = true;

	UDPData* data = reinterpret_cast<UDPData*>(recoveredData.data());
	auto payloadLength = recoveredData.size() - sizeof(UDPData::Header);

	if(data->header.size() > payloadLength)
	{
		ROS_ERROR("Invalid payload size in header (%u, received %lu), dropping", data->header.size(), payloadLength);
		return;
	}

	// Make sure the string fields are terminated
	data->header.topic_name[sizeof(data->header.topic_name)-1] = 0;
	data->header.topic_type[sizeof(data->header.topic_type)-1] = 0;

	auto output = std::make_shared<Message>();

	auto topicPtr = std::make_shared<Topic>(); // FIXME: Slightly awkward...
	topicPtr->name = data->header.topic_name;
	output->topic = topicPtr;
	output->type = data->header.topic_type;
	topic_info::unpackMD5(data->header.topic_md5, &output->md5);

	output->flags = data->header.flags();
	output->counter = data->header.topic_msg_counter;

	output->payload.resize(data->header.size());

	memcpy(
		output->payload.data(),
		data->data,
		output->payload.size()
	);

	// at this point, we can delete the packets to free up memory
	msg->packets.clear();

	// and off we go...
	m_cb(output);
}

void Depacketizer::pruneMessages()
{
	// First pass: Erase messages that are too old
	{
		auto itr = m_messageBuffer.begin();
		auto it_end = m_messageBuffer.end();
		for(int i = 0; i < 32; ++i)
		{
			itr++;
			if(itr == it_end)
				break;
		}

		m_messageBuffer.erase(itr, it_end);
	}

	// Second pass: erase messages that are more than 5s in the past
	{
		ros::SteadyTime cutoffTime = ros::SteadyTime::now() - ros::WallDuration(5.0);

		m_messageBuffer.remove_if([&](const PartialMessage& m) {
			return m.receptionTime < cutoffTime;
		});
	}
}

}
