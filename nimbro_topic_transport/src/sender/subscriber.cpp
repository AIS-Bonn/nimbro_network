// Start of the topic transport pipeline.
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "subscriber.h"

#include "../topic_info.h"

namespace nimbro_topic_transport
{

namespace
{
	constexpr double STAT_PERIOD = 60.0;

	class VectorBuffer
	{
	public:
		explicit VectorBuffer(std::vector<uint8_t>* vector)
		: m_vector(vector)
		{}

		inline uint8_t* advance(int off)
		{
			if(m_offset + off > m_vector->size())
				throw std::logic_error("VectorBuffer: long write");

			uint8_t* ptr = m_vector->data() + m_offset;
			m_offset += off;
			return ptr;
		}
	private:
		std::vector<uint8_t>* m_vector;
		std::size_t m_offset = 0;
	};
}

Subscriber::Subscriber(const Topic::Ptr& topic, ros::NodeHandle& nh, const std::string& fullTopicName)
 : m_topic(topic)
{
	// Transport hints
	ros::TransportHints hints;
	if(topic->config.hasMember("transport_hints"))
	{
		XmlRpc::XmlRpcValue list = topic->config["transport_hints"];
		if(list.getType() != XmlRpc::XmlRpcValue::TypeArray)
			throw std::runtime_error{"transport_hints should be a list"};

		for(int i = 0; i < list.size(); ++i)
		{
			XmlRpc::XmlRpcValue entry = list[i];
			if(entry.getType() != XmlRpc::XmlRpcValue::TypeString)
				throw std::runtime_error{"transport_hints should be a list of strings"};

			std::string s = entry;

			if(s == "udp")
				hints = hints.udp();
			else if(s == "unreliable")
				hints = hints.unreliable();
			else if(strcasecmp(s.c_str(), "nodelay") == 0 || strcasecmp(s.c_str(), "tcpnodelay") == 0)
				hints = hints.tcpNoDelay();
			else
				throw std::runtime_error{std::string{"Invalid transport_hint: "} + s};
		}
	}

	// Subscribe
	int queue_length = 1;
	if(topic->config.hasMember("queue"))
		queue_length = topic->config["queue"];

	ros::SubscribeOptions ops;
	boost::function<void(const ros::MessageEvent<topic_tools::ShapeShifter>&)> func
		= boost::bind(&Subscriber::handleData, this, _1);
	ops.initByFullCallbackType(fullTopicName, queue_length, func);
	ops.transport_hints = hints;

	m_subscriber = nh.subscribe(ops);

	// Initialize rate control
	if(topic->config.hasMember("rate"))
	{
		double rate = topic->config["rate"];
		      m_durationBetweenMsgs = ros::Duration(1.0 / rate);

		if(topic->config.hasMember("resend") && (bool)topic->config["resend"])
		{
			auto cb = std::bind(&Subscriber::resend, this);
			m_resendTimer = nh.createTimer(m_durationBetweenMsgs, cb);
		}
	}

	// Publisher filtering
	if(topic->config.hasMember("exclude_publishers"))
	{
		XmlRpc::XmlRpcValue list = topic->config["exclude_publishers"];
		if(list.getType() != XmlRpc::XmlRpcValue::TypeArray)
			throw std::runtime_error{"exclude_publishers should be a list"};

		for(int i = 0; i < list.size(); ++i)
		{
			XmlRpc::XmlRpcValue entry = list[i];
			if(entry.getType() != XmlRpc::XmlRpcValue::TypeString)
				throw std::runtime_error{"exclude_publishers should be a list of strings"};

			std::string s = entry;
			m_excludedPublishers.insert(ros::names::resolve(s));
		}
	}

	m_statTimer = nh.createSteadyTimer(ros::WallDuration(STAT_PERIOD), std::bind(&Subscriber::printStats, this));
}

void Subscriber::registerCallback(const Callback& cb)
{
	m_callbacks.push_back(cb);
}

void Subscriber::handleData(const ros::MessageEvent<topic_tools::ShapeShifter>& event)
{
	ROS_DEBUG("sender: message on topic '%s'", m_topic->name.c_str());
	m_incomingMessages++;

	// Is this publisher allowed?
	if(!m_excludedPublishers.empty())
	{
		auto& connHeader = event.getConnectionHeader();
		auto it = connHeader.find("callerid");

		if(it == connHeader.end())
			ROS_WARN_ONCE("No callerid in connection header");
		else
		{
			if(m_excludedPublishers.find(it->second) != m_excludedPublishers.end())
			{
				ROS_DEBUG_NAMED("filter", "Caller ID: '%s' in exclusion list, not sending", it->second.c_str());
				return;
			}
			else
				ROS_DEBUG_NAMED("filter", "Caller ID: '%s' not in exclusion list, sending", it->second.c_str());
		}
	}

	m_filteredMessages++;

	auto data = event.getMessage();
	auto msg = std::make_shared<Message>();

	// Serialize the shape shifter
	// TODO: Is there a way to get serialized data directly, avoiding the copy?
	{
		msg->payload.resize(data->size());
		VectorBuffer buf(&msg->payload);
		data->write(buf);
	}

	msg->topic = m_topic;
	msg->type = data->getDataType();
	msg->md5 = data->getMD5Sum();
	msg->counter = m_counter++;
	msg->receiveTime = event.getReceiptTime();

	m_lastMsg = msg;

	ros::Time now = ros::Time::now();
	if(now - m_lastTime < m_durationBetweenMsgs)
		return;

	m_lastTime = now;
	m_type = msg->type;
	m_md5 = msg->md5;

	for(auto& cb : m_callbacks)
		cb(msg);
}

void Subscriber::sendAdvertisement(const std::string& typeHint)
{
	if(m_type.empty() && !typeHint.empty())
	{
		m_type = typeHint;
		m_md5 = topic_info::getMd5Sum(typeHint);
	}

	if(m_type.empty() || m_md5.empty())
	{
		ROS_DEBUG("Cannot determine topic type of topic '%s'", rosTopicName().c_str());
		return;
	}

	ROS_DEBUG("Sending advertisement: %s", rosTopicName().c_str());

	auto msg = std::make_shared<Message>();
	msg->topic = m_topic;
	msg->type = m_type;
	msg->md5 = m_md5;
	msg->counter = m_counter++;
	msg->receiveTime = ros::Time::now();

	for(auto& cb : m_callbacks)
		cb(msg);
}

void Subscriber::resend()
{
	if(!m_lastMsg)
		return;

	ros::Time now = ros::Time::now();
	if(now - m_lastTime < m_durationBetweenMsgs)
		return;

	// TODO: Can we avoid the copy here? We do need to increment the counter, though...
	Message::Ptr msg(new Message(*m_lastMsg));
	msg->counter = m_counter++;
	msg->receiveTime = ros::Time::now();

	m_lastMsg = msg;

	for(auto& cb : m_callbacks)
		cb(msg);
}

void Subscriber::printStats()
{
	ROS_INFO("Topic %20s: Incoming %.2f Hz, filtered %.2f Hz",
		m_topic->name.c_str(),
		m_incomingMessages / STAT_PERIOD,
		m_filteredMessages / STAT_PERIOD
	);

	m_incomingMessages = 0;
	m_filteredMessages = 0;
}

}
