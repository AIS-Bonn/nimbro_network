// Top-level receiver class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_RECEIVER_H
#define TT_RECEIVER_H

#include "decompressor.h"
#include "depacketizer.h"
#include "publisher.h"
#include "tcp_receiver.h"
#include "udp_receiver.h"
#include "../thread_pool.h"

#include <memory>

namespace nimbro_topic_transport
{

class Receiver
{
public:
	Receiver(ros::NodeHandle nh = ros::NodeHandle("~"));
private:
	void handleMessage(const Message::ConstPtr& message);

	struct TopicHandler
	{
		explicit TopicHandler(const Topic::ConstPtr& topic, const std::string& prefix)
		 : topic(topic)
		 , publisher(topic, prefix)
		{}

		void handleMessage(const Message::ConstPtr& msg);

		Topic::ConstPtr topic;
		std::function<void(const Message::ConstPtr&)> queue;
		Decompressor decompressor;
		Publisher publisher;
	};

	ros::NodeHandle m_nh;

	Depacketizer m_depacketizer;
	ThreadPool m_threadPool;

	std::mutex m_topicMutex;
	std::map<std::string, std::unique_ptr<TopicHandler>> m_topics;

	TCPReceiver m_tcp_receiver;
	UDPReceiver m_udp_receiver;

	std::string m_prefix;
};

}

#endif
