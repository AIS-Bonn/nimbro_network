// Top-level receiver class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

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
	Receiver()
	{
		m_tcp_receiver.setCallback(std::bind(&Receiver::handleMessage, this, std::placeholders::_1));
		m_udp_receiver.setCallback(std::bind(&Depacketizer::addPacket, &m_depacketizer, std::placeholders::_1));

		m_depacketizer.setCallback(std::bind(&Receiver::handleMessage, this, std::placeholders::_1));
	}
private:
	void handleMessage(const Message::ConstPtr& message)
	{
		TopicHandler* handler;
		{
			std::unique_lock<std::mutex> lock(m_topicMutex);
			auto it = m_topics.find(message->topic->name);
			if(it == m_topics.end())
			{
				std::unique_ptr<TopicHandler> handlerPtr(new TopicHandler(message->topic));

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

	struct TopicHandler
	{
		TopicHandler(const Topic::ConstPtr& topic)
		 : topic(topic)
		 , publisher(topic)
		{}

		void handleMessage(const Message::ConstPtr& msg)
		{
			if(msg->flags & Message::FLAG_COMPRESSED_BZ2 || msg->flags & Message::FLAG_COMPRESSED_ZSTD)
			{
				auto decompressed = decompressor.decompress(msg);
				publisher.publish(decompressed);
			}
			else
				publisher.publish(msg);
		}

		Topic::ConstPtr topic;
		std::function<void(const Message::ConstPtr&)> queue;
		Decompressor decompressor;
		Publisher publisher;
	};

	Depacketizer m_depacketizer;
	ThreadPool m_threadPool;

	std::mutex m_topicMutex;
	std::map<std::string, std::unique_ptr<TopicHandler>> m_topics;

	TCPReceiver m_tcp_receiver;
	UDPReceiver m_udp_receiver;
};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "receiver");

	ros::NodeHandle nh("~");

	nimbro_topic_transport::Receiver receiver;

	ros::spin();

	return 0;
}
