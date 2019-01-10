// Test compression and decompression routines
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/sender/compressor.h"
#include "../../src/receiver/decompressor.h"

using namespace nimbro_topic_transport;

void testCompression(Compressor::Algorithm algo)
{
	auto topic = std::make_shared<Topic>();
	topic->name = "/hello_world";

	auto msg = std::make_shared<Message>();
	msg->counter = 10;
	msg->flags = 0;
	msg->type = "std_msgs/String";
	msg->md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
	msg->payload.resize(50 * 1024);
	for(std::size_t i = 0; i < msg->payload.size(); ++i)
		msg->payload[i] = i;

	auto compressor = std::make_shared<Compressor>(topic, 1, algo);
	auto compressed = compressor->compress(msg);

	if(algo == Compressor::Algorithm::ZSTD)
		REQUIRE((compressed->flags & Message::FLAG_COMPRESSED_ZSTD) != 0);
	else if(algo == Compressor::Algorithm::BZ2)
		REQUIRE((compressed->flags & Message::FLAG_COMPRESSED_BZ2) != 0);

	REQUIRE(compressed->payload.size() < msg->payload.size());

	auto decompressor = std::make_shared<Decompressor>();
	auto decompressed = decompressor->decompress(compressed);

	REQUIRE(msg->flags == decompressed->flags);
	REQUIRE(msg->type == decompressed->type);
	REQUIRE(msg->md5 == decompressed->md5);
	REQUIRE(msg->payload.size() == decompressed->payload.size());

	for(std::size_t i = 0; i < msg->payload.size(); ++i)
	{
		if(msg->payload[i] != decompressed->payload[i])
			FAIL("Payload mismatch at index " << i << ": expected: " << (int)msg->payload[i] << ", received: " << (int)decompressed->payload[i]);
	}
}

TEST_CASE("zstd", "[compression]")
{
	testCompression(Compressor::Algorithm::ZSTD);
}
