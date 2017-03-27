// Test packetization & depacketization for UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <catch_ros/catch.hpp>

#include <algorithm>

#include "../../src/sender/packetizer.h"
#include "../../src/receiver/depacketizer.h"

using namespace nimbro_topic_transport;

void testMessage(const Message::Ptr& msg, float fec)
{
	auto topic = std::make_shared<Topic>();
	topic->name = "/hello_world";
	topic->config["fec"] = fec;

	msg->topic = topic;

	auto packetizer = std::make_shared<Packetizer>();
	auto topicPacketizer = std::make_shared<TopicPacketizer>(packetizer, topic);

	auto packets = topicPacketizer->packetize(msg);

// 	printf("FEC %f => Got %d packets\n", fec, (int)packets.size());

	// Let's shuffle the packets
	{
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(packets.begin(), packets.end(), g);
	}

	// And drop some
	{
		std::size_t sourceSymbols = packets.size() / (1.0f + fec);
		std::size_t drop_surplus = (fec / 2.0f) * sourceSymbols;
// 		printf("Dropping %lu packets\n", drop_surplus);
		packets.resize(packets.size() - drop_surplus);
	}

	auto depacketizer = std::make_shared<Depacketizer>();

	Message::ConstPtr receivedMsg;

	depacketizer->setCallback([&](const Message::ConstPtr& newMsg){
		REQUIRE(newMsg->topic->name == msg->topic->name);
		REQUIRE(!receivedMsg);
		receivedMsg = newMsg;
	});

	for(auto& packet : packets)
		depacketizer->addPacket(packet);

	REQUIRE(receivedMsg);
	REQUIRE(msg->flags == receivedMsg->flags);
	REQUIRE(msg->type == receivedMsg->type);
	REQUIRE(msg->md5 == receivedMsg->md5);
	REQUIRE(msg->payload.size() == receivedMsg->payload.size());

	for(std::size_t i = 0; i < msg->payload.size(); ++i)
	{
		if(msg->payload[i] != receivedMsg->payload[i])
			FAIL("Payload mismatch at index " << i << ": expected: " << (int)msg->payload[i] << ", received: " << (int)receivedMsg->payload[i]);
	}
}

TEST_CASE("big", "[packetizer]")
{
	auto msg = std::make_shared<Message>();
	msg->counter = 10;
	msg->flags = Message::FLAG_COMPRESSED_ZSTD;
	msg->type = "std_msgs/String";
	msg->md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
	msg->payload.resize(50 * 1024);
	for(std::size_t i = 0; i < msg->payload.size(); ++i)
		msg->payload[i] = i;

	for(int i = 0; i < 5; ++i)
		testMessage(msg, 0.0);
	for(int i = 0; i < 5; ++i)
		testMessage(msg, 0.5);
}

TEST_CASE("small", "[packetizer]")
{
	auto msg = std::make_shared<Message>();
	msg->counter = 10;
	msg->flags = Message::FLAG_COMPRESSED_ZSTD;
	msg->type = "std_msgs/String";
	msg->md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
	msg->payload.resize(200);
	for(std::size_t i = 0; i < msg->payload.size(); ++i)
		msg->payload[i] = i;

	for(int i = 0; i < 5; ++i)
		testMessage(msg, 0.0);
	for(int i = 0; i < 5; ++i)
		testMessage(msg, 0.5);
}

TEST_CASE("increasing", "[packetizer]")
{
	for(std::size_t size = 1; size < 2048; ++size)
	{
		auto msg = std::make_shared<Message>();
		msg->counter = 10;
		msg->flags = Message::FLAG_COMPRESSED_ZSTD;
		msg->type = "std_msgs/String";
		msg->md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
		msg->payload.resize(size);
		for(std::size_t i = 0; i < msg->payload.size(); ++i)
			msg->payload[i] = i;

		testMessage(msg, 0.5);
	}
}

TEST_CASE("huge", "[packetizer]")
{
	auto msg = std::make_shared<Message>();
	msg->counter = 10;
	msg->flags = Message::FLAG_COMPRESSED_ZSTD;
	msg->type = "std_msgs/String";
	msg->md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
	msg->payload.resize(5 * 1024 * 1024);
	for(std::size_t i = 0; i < msg->payload.size(); ++i)
		msg->payload[i] = i;

	for(int i = 0; i < 5; ++i)
		testMessage(msg, 0.0);
	for(int i = 0; i < 5; ++i)
		testMessage(msg, 0.5);
}
