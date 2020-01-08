// Top-level control for the sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "sender.h"

namespace nimbro_topic_transport
{

namespace
{

namespace String
{
	bool beginsWith(const std::string_view& string, const std::string_view& prefix)
	{
		if(string.size() < prefix.size())
			return false;

		return std::strncmp(string.data(), prefix.data(), prefix.size()) == 0;
	}
}

}

Sender::Sender()
 : m_nh("~")
{
	m_nh.getParam("strip_prefix", m_stripPrefix);

	if(m_nh.hasParam("tcp_topics"))
	{
		XmlRpc::XmlRpcValue topicList;
		m_nh.getParam("tcp_topics", topicList);

		initTCP(topicList);
	}

	if(m_nh.hasParam("udp_topics"))
	{
		XmlRpc::XmlRpcValue topicList;
		m_nh.getParam("udp_topics", topicList);

		initUDP(topicList);
	}

	ROS_INFO("Sender initialized, listening on %lu topics.", m_subs.size());
}

std::string Sender::stripPrefix(const std::string& topic) const
{
	if(String::beginsWith(topic, m_stripPrefix))
		return topic.substr(m_stripPrefix.size());
	else
		return topic;
}

void Sender::initTCP(XmlRpc::XmlRpcValue& topicList)
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

		std::string fullName = static_cast<std::string>(entry["name"]);
		topic->name = stripPrefix(fullName);
		topic->config = entry;

		std::unique_ptr<Subscriber> sub(new Subscriber(topic, m_nh, fullName));

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

void Sender::initUDP(XmlRpc::XmlRpcValue& topicList)
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

		std::string fullName = static_cast<std::string>(entry["name"]);
		topic->name = stripPrefix(fullName);
		topic->config = entry;

		std::unique_ptr<Subscriber> sub(new Subscriber(topic, m_nh, fullName));

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
			sink = [packetizer,this](const Message::ConstPtr& msg) {
				m_udp_sender->send(packetizer->packetize(msg));
			};
		}

		sub->registerCallback(m_threadPool.createInputHandler(sink));
		m_subs.emplace_back(std::move(sub));
	}
}

}
