// UDP packet definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_PACKET_H
#define UDP_PACKET_H

#include "../le_value.h"

namespace sb_topic_transport
{

const int PACKET_SIZE = 1024;

enum PacketType
{
	PACKET_INFO,
	PACKET_DATA
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

}

#endif
