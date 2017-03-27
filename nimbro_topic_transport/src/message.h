// Metadata for a single message
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_MESSAGE_H
#define TT_MESSAGE_H

#include <topic_tools/shape_shifter.h>

#include "topic.h"

namespace nimbro_topic_transport
{

class Message
{
public:
	typedef std::shared_ptr<Message> Ptr;
	typedef std::shared_ptr<const Message> ConstPtr;

	enum Flag
	{
		FLAG_COMPRESSED_BZ2      = (1 << 0),
		FLAG_COMPRESSED_ZSTD     = (1 << 1),
	};

	void copyMetaInfoFrom(const Message& other)
	{
		topic = other.topic;
		type = other.type;
		md5 = other.md5;
		flags = other.flags;
		counter = other.counter;
	}

	std::shared_ptr<const Topic> topic;
	std::string type;
	std::string md5;
	uint16_t flags = 0;
	std::vector<uint8_t> payload;

	//! Sequential counter (incremented in Subscriber class)
	uint32_t counter;
};

}

#endif
