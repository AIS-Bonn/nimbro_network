// UDP packet definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_PACKET_H
#define UDP_PACKET_H

#include "../le_value.h"

namespace nimbro_topic_transport
{

const int PACKET_SIZE = 1500 - 20 - 8;

enum PacketType
{
	PACKET_INFO,
	PACKET_DATA
};

enum UDPFlag
{
	UDP_FLAG_COMPRESSED = (1 << 0),
	UDP_FLAG_RELAY_MODE = (1 << 1)
};

struct UDPGenericPacket
{
	LEValue<2> frag_id;
	LEValue<2> msg_id;
} __attribute__((packed));

struct UDPFirstPacket
{
	struct Header
	{
		LEValue<2> frag_id;
		LEValue<2> msg_id;
		char topic_name[64];
		char topic_type[64];
		LEValue<4> topic_md5[4];
		LEValue<2> remaining_packets;
		LEValue<2> flags;
		LEValue<2> topic_msg_counter;
	} __attribute__((packed));

	enum { MaxDataSize = PACKET_SIZE - sizeof(Header) };

	Header header;
	uint8_t data[];
} __attribute__((packed));

struct UDPDataPacket
{
	struct Header
	{
		LEValue<2> frag_id;
		LEValue<2> msg_id;
	} __attribute__((packed));

	enum { MaxDataSize = PACKET_SIZE - sizeof(Header) };

	Header header;
	uint8_t data[];
} __attribute__((packed));

// Minimum number of packets for choosing the LDPC-Staircase algorithm
const int MIN_PACKETS_LDPC = 1000;

struct FECHeader
{
	char topic_name[64];
	char topic_type[64];
	LEValue<4> topic_md5[4];
	LEValue<2> flags;
	LEValue<2> topic_msg_counter;

	uint8_t data[];
} __attribute__((packed));

struct FECPacket
{
	struct Header
	{
		LEValue<2> msg_id;
		LEValue<4> symbol_id;
		LEValue<2> symbol_length;
		LEValue<2> source_symbols;
		LEValue<2> repair_symbols;
	} __attribute__((packed));

	enum { MaxDataSize = PACKET_SIZE - sizeof(Header) };

	Header header;
	uint8_t data[];
} __attribute__((packed));

}

#endif
