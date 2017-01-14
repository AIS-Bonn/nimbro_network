// Decompress compressed messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "decompressor.h"

#include <bzlib.h>

#if WITH_ZSTD
#include <zstd.h>
#endif

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
		auto result = std::make_shared<Message>();

		unsigned int destLen = 1024;

		result->payload.resize(destLen);

		while(1)
		{
			int ret = BZ2_bzBuffToBuffDecompress(
				(char*)result->payload.data(), &destLen,
				(char*)msg->payload.data(), msg->payload.size(),
				0, 0
			);

			if(ret == BZ_OK)
				break;
			else if(ret == BZ_OUTBUFF_FULL)
			{
				destLen *= 2;
				result->payload.resize(destLen);
			}
			else
			{
				ROS_ERROR_THROTTLE(1.0,
					"Could not decompress message using BZ2. Dropping..."
				);

				return Message::ConstPtr();
			}
		}

		result->payload.resize(destLen);
		result->copyMetaInfoFrom(*msg);
		result->flags &= ~Message::FLAG_COMPRESSED_BZ2;

		return result;
	}
	else if(msg->flags & Message::FLAG_COMPRESSED_ZSTD)
	{
#if WITH_ZSTD
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
#else
		ROS_ERROR_THROTTLE(1.0,
			"Got ZSTD compressed message, but nimbro_topic_transport was not "
			"compiled with ZSTD support. Dropping message."
		);

		return Message::ConstPtr();
#endif
	}
	else
		return msg;
}

}
