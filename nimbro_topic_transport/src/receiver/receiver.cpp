// Top-level receiver class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "receiver.h"

namespace nimbro_topic_transport
{

Receiver::Receiver(ros::NodeHandle nh)
 : m_nh{std::move(nh)}
 , m_tcp_receiver{m_nh}
 , m_udp_receiver{m_nh}
{
	m_nh.getParam("prefix", m_prefix);

	m_rewriter = std::make_unique<Rewriter>(m_prefix);

	m_tcp_receiver.setCallback(std::bind(&Receiver::handleMessage, this, std::placeholders::_1));
	m_udp_receiver.setCallback(std::bind(&Depacketizer::addPacket, &m_depacketizer, std::placeholders::_1));

	m_depacketizer.setCallback(std::bind(&Receiver::handleMessage, this, std::placeholders::_1));
}

void Receiver::handleMessage(const Message::ConstPtr& message)
{
	TopicHandler* handler;
	{
		std::unique_lock<std::mutex> lock(m_topicMutex);
		auto it = m_topics.find(message->topic->name);
		if(it == m_topics.end())
		{
			std::unique_ptr<TopicHandler> handlerPtr(new TopicHandler(message->topic, *m_rewriter));

			handlerPtr->queue = m_threadPool.createInputHandler(
				std::bind(&TopicHandler::handleMessage, handlerPtr.get(), std::placeholders::_1)
			);

			auto ret = m_topics.emplace(message->topic->name, std::move(handlerPtr));
			it = ret.first;
		}

		handler = it->second.get();
	}

	handler->queue(message);
}

void Receiver::TopicHandler::handleMessage(const Message::ConstPtr& msg)
{
	if(msg->flags & Message::FLAG_COMPRESSED_BZ2 || msg->flags & Message::FLAG_COMPRESSED_ZSTD)
	{
		auto decompressed = decompressor.decompress(msg);
		publisher.publish(decompressed);
	}
	else
		publisher.publish(msg);
}

}
