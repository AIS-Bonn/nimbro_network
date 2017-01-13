// UDP packet definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_PACKET_H
#define UDP_PACKET_H

#include "le_value.h"

namespace nimbro_topic_transport
{

// Since we sometimes tunnel nimbro_network packets through QuickTun,
// leave space for one additional UDP/IP envelope.
const int PACKET_SIZE = 1500 - 20 - 8 - 20 - 8;

struct UDPPacket
{
	struct Header
	{
		LEValue<2> msg_id;
		LEValue<4> symbol_id;

		// FEC parameters
		LEValue<2> source_symbols;
		LEValue<2> repair_symbols; //!< if zero, no FEC.
	} __attribute__((packed));

	enum { MaxDataSize = PACKET_SIZE - sizeof(Header) };

	Header header;
	uint8_t data[];
} __attribute__((packed));

//! Header of the data (either after FEC decoding or in first packet)
struct UDPData
{
	struct Header
	{
		char topic_name[64];
		char topic_type[64];
		LEValue<4> topic_md5[4];
		LEValue<2> flags;
		LEValue<2> topic_msg_counter;
		LEValue<4> size;
	};

	enum { MaxDataSize = UDPPacket::MaxDataSize - sizeof(Header) };

	Header header;
	uint8_t data[];
} __attribute__((packed));

}

#endif
