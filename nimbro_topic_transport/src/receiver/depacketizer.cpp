// De-fragments packets into complete messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "depacketizer.h"

#include <algorithm>

#include <ros/console.h>

extern "C"
{
#include <of_openfec_api.h>
}

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

	uint64_t totalSymbols = header.source_symbols() + header.repair_symbols();
	msg->packets.resize(totalSymbols);

	msg->packets[header.symbol_id()] = packet;

	std::size_t symbolSize;
	if(header.source_symbols() == 1)
		symbolSize = packet->length - sizeof(UDPPacket::Header);
	else
		symbolSize = UDPPacket::MaxDataSize;

	if(msg->decoder)
	{
		if(header.source_symbols() != msg->params->nb_source_symbols
			|| header.repair_symbols() != msg->params->nb_repair_symbols
			|| symbolSize != msg->params->encoding_symbol_length)
		{
			ROS_ERROR("Received packet with mismatched FEC parameters, dropping");
			return;
		}
	}

	// Do we need to do FEC decoding?
	if(header.repair_symbols())
	{
		if(!msg->decoder)
		{
			of_session_t* ses = 0;
			of_parameters_t* params = 0;

			if(header.source_symbols() >= MIN_PACKETS_LDPC)
			{
				ROS_DEBUG("LDPC");
				if(of_create_codec_instance(&ses, OF_CODEC_LDPC_STAIRCASE_STABLE, OF_DECODER, 1) != OF_STATUS_OK)
				{
					ROS_ERROR("Could not create LDPC decoder");
					return;
				}

				of_ldpc_parameters_t* ldpc_params = (of_ldpc_parameters_t*)malloc(sizeof(of_ldpc_parameters_t));
				ldpc_params->nb_source_symbols = header.source_symbols();
				ldpc_params->nb_repair_symbols = header.repair_symbols();
				ldpc_params->encoding_symbol_length = symbolSize;
				ldpc_params->prng_seed = header.prng_seed();
				ldpc_params->N1 = 7;

				ROS_DEBUG("LDPC parameters: %d, %d, %d, 0x%X, %d", ldpc_params->nb_source_symbols, ldpc_params->nb_repair_symbols, ldpc_params->encoding_symbol_length, ldpc_params->prng_seed, ldpc_params->N1);

				params = (of_parameters*)ldpc_params;
			}
			else
			{
				ROS_DEBUG("REED");
				if(of_create_codec_instance(&ses, OF_CODEC_REED_SOLOMON_GF_2_M_STABLE, OF_DECODER, 1) != OF_STATUS_OK)
				{
					ROS_ERROR("Could not create REED_SOLOMON decoder");
					return;
				}

				of_rs_2_m_parameters* rs_params = (of_rs_2_m_parameters_t*)malloc(sizeof(of_rs_2_m_parameters_t));
				rs_params->nb_source_symbols = header.source_symbols();
				rs_params->nb_repair_symbols = header.repair_symbols();
				rs_params->encoding_symbol_length = symbolSize;
				rs_params->m = 8;

				ROS_DEBUG("REED params: %d, %d, %d", rs_params->nb_source_symbols, rs_params->nb_repair_symbols, rs_params->encoding_symbol_length);

				params = (of_parameters_t*)rs_params;
			}

			if(of_set_fec_parameters(ses, params) != OF_STATUS_OK)
			{
				ROS_ERROR("Could not set FEC parameters");
				of_release_codec_instance(ses);
				return;
			}

			msg->decoder.reset(ses, of_release_codec_instance);
			msg->params.reset(params, free);
		}

		if(of_decode_with_new_symbol(msg->decoder.get(), const_cast<uint8_t*>(packet->packet()->data), header.symbol_id()) != OF_STATUS_OK)
		{
			ROS_ERROR_THROTTLE(5.0, "Could not decode symbol");
			return;
		}
	}

	msg->received_symbols++;

	bool done = false;
	if(msg->received_symbols >= header.source_symbols())
	{
		if(header.repair_symbols() == 0)
		{
			// No FEC
			done = true;
		}
		else
		{
			done = of_is_decoding_complete(msg->decoder.get());

			// As it is implemented in OpenFEC right now, we can only
			// try the ML decoding (gaussian elimination) *once*.
			// After that the internal state is screwed up and the message
			// has to be discarded.
			// So we have to be sure that it's worth it ;-)
			if(!done && msg->received_symbols >= msg->params->nb_source_symbols + msg->params->nb_repair_symbols / 2)
			{
				of_status_t ret = of_finish_decoding(msg->decoder.get());
				if(ret == OF_STATUS_OK)
					done = true;
				else
				{
					ROS_ERROR("ML decoding failed, dropping message...");
					msg->complete = true;
					return;
				}
			}
		}
	}

	if(done)
	{
		msg->complete = true;

		std::vector<void*> symbols(header.source_symbols, 0);

		if(header.repair_symbols() != 0)
		{
			if(of_get_source_symbols_tab(msg->decoder.get(), symbols.data()) != OF_STATUS_OK)
			{
				ROS_ERROR("Could not get decoded symbols, dropping");
				return;
			}
		}
		else
		{
			for(std::size_t i = 0; i < header.source_symbols(); ++i)
			{
				if(!msg->packets[i])
				{
					ROS_ERROR("Missing packet in complete non-FEC message, is there a wrong index?");
					return;
				}

				symbols[i] = msg->packets[i]->packet()->data;
			}
		}

		uint64_t payloadLength = header.source_symbols() * symbolSize;
		if(symbolSize < sizeof(UDPData) || payloadLength < sizeof(UDPData))
		{
			ROS_ERROR("Invalid short payload, dropping");
			return;
		}

		UDPData::Header msgHeader;
		memcpy(&msgHeader, symbols[0], sizeof(msgHeader));
		payloadLength -= sizeof(msgHeader);

		if(msgHeader.size() > payloadLength)
		{
			ROS_ERROR("Invalid payload size in header (%u, received %lu), dropping", msgHeader.size(), payloadLength);
			return;
		}

		// Make sure the string fields are terminated
		msgHeader.topic_name[sizeof(msgHeader.topic_name)-1] = 0;
		msgHeader.topic_type[sizeof(msgHeader.topic_type)-1] = 0;

		auto output = std::make_shared<Message>();

		auto topicPtr = std::make_shared<Topic>(); // FIXME: Slightly awkward...
		topicPtr->name = msgHeader.topic_name;
		output->topic = topicPtr;
		output->type = msgHeader.topic_type;
		topic_info::unpackMD5(msgHeader.topic_md5, &output->md5);

		output->flags = msgHeader.flags();
		output->counter = msgHeader.topic_msg_counter;

		output->payload.resize(payloadLength);
		uint8_t* writePtr = output->payload.data();

		// Read 1st symbol payload
		memcpy(
			writePtr,
			reinterpret_cast<UDPData*>(symbols[0])->data,
			symbolSize - sizeof(UDPData::Header)
		);
		writePtr += symbolSize - sizeof(UDPData::Header);

		// Read remaining symbols
		for(unsigned int symbol = 1; symbol < symbols.size(); ++symbol)
		{
			memcpy(writePtr, symbols[symbol], symbolSize);
			writePtr += symbolSize;
		}

		// at this point, we can delete the packets to free up memory
		msg->packets.clear();

		// Resize to remove FEC padding
		output->payload.resize(msgHeader.size());

		// and off we go...
		m_cb(output);
	}
}

void Depacketizer::pruneMessages()
{
	// Erase messages that are too old (after index 31)
	auto itr = m_messageBuffer.begin();
	auto it_end = m_messageBuffer.end();
	for(int i = 0; i < 31; ++i)
	{
		itr++;
		if(itr == it_end)
			break;
	}
}

}
