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

Sender::Sender(ros::NodeHandle nh)
 : m_nh{std::move(nh)}
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

	double topicRefreshRate = 0.25;
	m_nh.getParam("topic_refresh_rate", topicRefreshRate);

	double advertisementRate = 1.0;
	m_nh.getParam("topic_advertisement_rate", advertisementRate);

	ROS_INFO("Topic refresh rate: %f Hz, advertisement rate: %f Hz", topicRefreshRate, advertisementRate);

	m_advertiseTimer = m_nh.createSteadyTimer(ros::WallDuration(1.0/advertisementRate), std::bind(&Sender::advertiseTopics, this));
	m_topicThread = std::thread(std::bind(&Sender::refreshTopicList, this, topicRefreshRate));

	ROS_INFO("Sender initialized, listening on %lu topics.", m_subs.size());
}

Sender::~Sender()
{
	m_topicThread.join();
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
	m_tcp_sender = std::make_unique<TCPSender>(m_nh);

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
	m_udp_sender = std::make_unique<UDPSender>(m_nh);
	m_packetizer = std::make_unique<Packetizer>();

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

void Sender::advertiseTopics()
{
	std::unique_lock<std::mutex> lock(m_topicTypeMapMutex);

	for(auto& sub : m_subs)
	{
		auto it = m_topicTypeMap.find(sub->rosTopicName());

		if(it != m_topicTypeMap.end())
			sub->sendAdvertisement(it->second);
		else
			sub->sendAdvertisement();
	}
}

void Sender::refreshTopicList(double _rate)
{
	ros::WallRate rate(_rate);

	for(; ros::ok(); rate.sleep())
	{
		std::vector<ros::master::TopicInfo> topicInfos;
		if(!ros::master::getTopics(topicInfos))
		{
			ROS_ERROR_THROTTLE(1.0, "Could not query topics from master");
			continue;
		}

		std::map<std::string, std::string> update;
		for(auto& topic : topicInfos)
		{
			update[topic.name] = topic.datatype;
		}

		{
			std::unique_lock<std::mutex> lock(m_topicTypeMapMutex);
			m_topicTypeMap = std::move(update);
		}
	}
}

}
