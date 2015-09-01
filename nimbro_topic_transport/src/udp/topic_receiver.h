// Topic receiver (part of udp_receiver)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPIC_RECEIVER_H
#define TOPIC_RECEIVER_H

#include <stdint.h>
#include <stdlib.h>

#include <vector>

#include <ros/publisher.h>

#include <topic_tools/shape_shifter.h>

#include <nimbro_topic_transport/CompressedMsg.h>

#include <boost/thread.hpp>

#if WITH_OPENFEC
extern "C"
{
#include <of_openfec_api.h>
}
#endif

#include "udp_packet.h"

namespace nimbro_topic_transport
{

struct Message
{
	Message(uint16_t id)
	 : id(id)
	 , size(0)
	 , complete(false)
	{}

	Message()
	{}

	~Message()
	{
	}

	uint32_t getLength() const
	{ return size; }

	uint8_t* getData()
	{ return payload.data(); }

	bool decompress(Message* dest);

	uint16_t id;
	UDPFirstPacket::Header header;
	std::vector<uint8_t> payload;
	size_t size;
	std::vector<bool> msgs;

	bool complete;

#if WITH_OPENFEC
	boost::shared_ptr<of_session_t> decoder;
	boost::shared_ptr<of_parameters_t> params;
	unsigned int received_symbols;
	std::vector<boost::shared_ptr<std::vector<uint8_t>>> fecPackets;
#endif
};

struct TopicReceiver
{
	TopicReceiver();
	~TopicReceiver();

	ros::Publisher publisher;

	uint32_t md5[4];
	std::string md5_str;
	std::string msg_def;

	bool compressed;

	int last_message_counter;

	bool waiting_for_subscriber;

	void takeForDecompression(const boost::shared_ptr<Message>& compressed);

	void publish(const boost::shared_ptr<topic_tools::ShapeShifter>& msg);
	void publishCompressed(const CompressedMsgConstPtr& msg);
	void handleSubscriber();
private:
	boost::shared_ptr<Message> m_compressedMsg;
	boost::condition_variable m_cond;
	boost::mutex m_mutex;

	boost::thread m_decompressionThread;
	bool m_decompressionThreadRunning;
	bool m_decompressionThreadShouldExit;

	boost::shared_ptr<topic_tools::ShapeShifter> m_msgInQueue;
	CompressedMsgConstPtr m_compressedMsgInQueue;
	ros::WallTime m_queueTime;

	void decompress();
};

}

#endif
