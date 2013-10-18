// TCP packet definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_PACKET_H
#define TCP_PACKET_H

#include "../le_value.h"

namespace sb_topic_transport
{

struct TCPHeader
{
	LEValue<2> topic_len;
	LEValue<2> type_len;
	LEValue<4> data_len;
	LEValue<4> topic_md5sum[4];
} __attribute__((packed));

}

#endif
