// Decompress compressed messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "decompressor.h"

#include <bzlib.h>

#include <zstd.h>

namespace nimbro_topic_transport
{

Decompressor::Decompressor()
{
}

Decompressor::~Decompressor()
{
}

Message::ConstPtr Decompressor::decompress(const Message::ConstPtr& msg)
{
	if(msg->flags & Message::FLAG_COMPRESSED_BZ2)
	{
		ROS_ERROR("Got BZ2 compressed message. BZ2 is obsolete, dropping. Please update the sender!");
		return Message::ConstPtr();
	}
	else if(msg->flags & Message::FLAG_COMPRESSED_ZSTD)
	{
		auto result = std::make_shared<Message>();

		std::size_t destLen = ZSTD_getDecompressedSize(msg->payload.data(), msg->payload.size());
		if(destLen == 0 || destLen >= (1 << 28)) // 256 MiB
		{
			ROS_WARN_THROTTLE(1.0,
				"Could not infer decompressed size. "
				"Maybe your message is larger than 256 MiB?"
			);
			destLen = 5ULL * 1024ULL * 1024ULL; // 5MiB
		}

		result->payload.resize(destLen);

		std::size_t size = ZSTD_decompress(
			result->payload.data(), destLen,
			msg->payload.data(), msg->payload.size()
		);

		if(ZSTD_isError(size))
		{
			ROS_ERROR_THROTTLE(1.0,
				"Could not decompress message using ZSTD: %s",
				ZSTD_getErrorName(size)
			);
			return Message::ConstPtr();
		}

		result->copyMetaInfoFrom(*msg);
		result->flags &= ~Message::FLAG_COMPRESSED_ZSTD;

		return result;
	}
	else
		return msg;
}

}
