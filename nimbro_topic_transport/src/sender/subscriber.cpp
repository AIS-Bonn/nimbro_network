// Start of the topic transport pipeline.
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "subscriber.h"

#include "../topic_info.h"

namespace nimbro_topic_transport
{

namespace
{

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

Subscriber::Subscriber(const Topic::Ptr& topic, ros::NodeHandle& nh)
 : m_topic(topic)
{
	// Subscribe
	int queue_length = 1;
	if(topic->config.hasMember("queue"))
		queue_length = topic->config["queue"];

	ros::SubscribeOptions ops;
	boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> func
		= boost::bind(&Subscriber::handleData, this, _1);
	ops.initByFullCallbackType(topic->name, queue_length, func);

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
}

void Subscriber::registerCallback(const Callback& cb)
{
	m_callbacks.push_back(cb);
}

void Subscriber::handleData(const topic_tools::ShapeShifter::ConstPtr& data)
{
	ROS_DEBUG("sender: message on topic '%s'", m_topic->name.c_str());
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

	m_lastMsg = msg;

	ros::Time now = ros::Time::now();
	if(now - m_lastTime < m_durationBetweenMsgs)
		return;

	m_lastTime = now;

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

	m_lastMsg = msg;

	for(auto& cb : m_callbacks)
		cb(msg);
}

}
