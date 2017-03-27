// Top-level control for the sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/node_handle.h>

#include "subscriber.h"
#include "tcp_sender.h"
#include "udp_sender.h"
#include "packetizer.h"
#include "compressor.h"
#include "thread_pool.h"

namespace nimbro_topic_transport
{

class Sender
{
public:
	Sender()
	 : m_nh("~")
	{
	}

private:
	void initTCP(XmlRpc::XmlRpcValue& topicList)
	{
		m_tcp_sender.reset(new TCPSender);

		for(int32_t i = 0; i < topicList.size(); ++i)
		{
			XmlRpc::XmlRpcValue& entry = topicList[i];

			auto topic = std::make_shared<Topic>();

			if(entry.getType() != XmlRpc::XmlRpcValue::TypeStruct)
				throw std::logic_error("Topic list entries have to be structs");
			if(!entry.hasMember("name"))
				throw std::logic_error("Topic list entries have to have a name");

			topic->name = static_cast<std::string>(entry["name"]);
			topic->config = entry;

			std::unique_ptr<Subscriber> sub(new Subscriber(topic, m_nh));

			unsigned int level = Compressor::getCompressionLevel(*topic);
			if(level != 0)
			{
				auto compressor = std::make_shared<Compressor>(topic, level);

				auto cb = [compressor,this](const Message::ConstPtr& msg) {
					m_tcp_sender->send(compressor->compress(msg));
				};

				sub->registerCallback(m_threadPool.createInputHandler(cb));
			}
			else
			{
				// send directly
				sub->registerCallback([&](const Message::ConstPtr& msg) {
					m_tcp_sender->send(msg);
				});
			}

			m_subs.emplace_back(std::move(sub));
		}
	}

	void initUDP(XmlRpc::XmlRpcValue& topicList)
	{
		m_udp_sender.reset(new UDPSender);
		m_packetizer.reset(new Packetizer);

		for(int32_t i = 0; i < topicList.size(); ++i)
		{
			XmlRpc::XmlRpcValue& entry = topicList[i];

			auto topic = std::make_shared<Topic>();

			if(entry.getType() != XmlRpc::XmlRpcValue::TypeStruct)
				throw std::logic_error("Topic list entries have to be structs");
			if(!entry.hasMember("name"))
				throw std::logic_error("Topic list entries have to have a name");

			topic->name = static_cast<std::string>(entry["name"]);
			topic->config = entry;

			std::unique_ptr<Subscriber> sub(new Subscriber(topic, m_nh));

			auto packetizer = std::make_shared<TopicPacketizer>(m_packetizer, topic);

			std::function<void(const Message::ConstPtr&)> sink;
			unsigned int level = Compressor::getCompressionLevel(*topic);
			if(level != 0)
			{
				auto compressor = std::make_shared<Compressor>(topic, level);

				// Compress, packetize, and send
				sink = [compressor,packetizer,this](const Message::ConstPtr& msg) {
					m_udp_sender->send(packetizer->packetize(compressor->compress(msg)));
				};
			}
			else
			{
				// Packetize and send
				sink = [&](const Message::ConstPtr& msg) {
					m_udp_sender->send(packetizer->packetize(msg));
				};
			}

			sub->registerCallback(m_threadPool.createInputHandler(sink));
			m_subs.emplace_back(std::move(sub));
		}
	}

	ros::NodeHandle m_nh;
	std::vector<std::unique_ptr<Subscriber>> m_subs;
	std::unique_ptr<TCPSender> m_tcp_sender;

	std::unique_ptr<UDPSender> m_udp_sender;

	ThreadPool m_threadPool;
	std::shared_ptr<Packetizer> m_packetizer;
};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sender");

	nimbro_topic_transport::Sender sender;

	ros::spin();

	return 0;
}
