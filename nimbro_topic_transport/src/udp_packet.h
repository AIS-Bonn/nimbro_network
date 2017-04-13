// UDP packet definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_PACKET_H
#define UDP_PACKET_H

#include "le_value.h"

#include <array>
#include <memory>

namespace nimbro_topic_transport
{

// Since we sometimes tunnel nimbro_network packets through QuickTun,
// leave space for one additional UDP/IP envelope.
const int PACKET_SIZE = 1500 - 20 - 8 - 20 - 8;

//! Minimum number of packets for choosing LDPC FEC algorithm
//! Below this limit, Reed-Solomon is used.
const int MIN_PACKETS_LDPC = 255;

struct UDPPacket
{
	struct Header
	{
		LEValue<3> packet_id;
		LEValue<2> msg_id;
		LEValue<3> symbol_id;

		// FEC parameters
		LEValue<2> source_symbols;
		LEValue<2> repair_symbols; //!< if zero, no FEC.
		LEValue<4> prng_seed;

		// Padding to reach 16 bytes alignment for the payload.
		// This makes FEC computations much more efficient at a small
		// bandwidth cost.
		// In particular, OpenFEC silently assumes 8-bit alignment of the
		// symbols - see unprotected casts in of_symbol.c.
		// NOTE: If you change the header, make sure its size is a multiple of 16!
		// current size: 16 bytes.
	} __attribute__((packed));

	static_assert(sizeof(Header) % 16 == 0, "UDP Header size must be a multiple of 16");

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
	} __attribute__((packed));

	enum { MaxDataSize = UDPPacket::MaxDataSize - sizeof(Header) };

	Header header;
	uint8_t data[];
} __attribute__((packed));

//! High-level packet buffer
class Packet
{
public:
	typedef std::shared_ptr<Packet> Ptr;
	typedef std::shared_ptr<const Packet> ConstPtr;

	inline UDPPacket* packet()
	{ return reinterpret_cast<UDPPacket*>(data.data()); }

	inline const UDPPacket* packet() const
	{ return reinterpret_cast<const UDPPacket*>(data.data()); }

	std::array<uint8_t, PACKET_SIZE> data;
	std::size_t length;
};

}

#endif
