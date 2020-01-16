// Publish received messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "publisher.h"

#include "../topic_info.h"

namespace
{

class VectorStream
{
public:
	VectorStream(const std::vector<uint8_t>* vector)
	 : m_vector(vector)
	{}

	inline const uint8_t* getData()
	{ return m_vector->data(); }

	inline size_t getLength()
	{ return m_vector->size(); }
private:
	const std::vector<uint8_t>* m_vector;
};

}

namespace nimbro_topic_transport
{

Publisher::Publisher(const Topic::ConstPtr& topic, Rewriter& rewriter)
 : m_rewriter{rewriter}
{
}

Publisher::~Publisher()
{
}

void Publisher::publish(const Message::ConstPtr& msg)
{
	ROS_DEBUG("Publisher::publish() for topic '%s'", msg->topic->name.c_str());

	std::unique_lock<std::mutex> lock(m_holdoffMutex);

	if(!m_advertised)
	{
		ROS_INFO("Advertising new topic '%s'", msg->topic->name.c_str());
		m_messageDefinition = topic_info::getMsgDef(msg->type);

		ros::NodeHandle nh;

		ros::AdvertiseOptions options(
			m_rewriter.rewriteTopicName(msg->topic->name),
			50,
			msg->md5,
			msg->type,
			m_messageDefinition
		);

		m_pub = nh.advertise(options);

		// Because subscribers will not immediately subscribe, we introduce
		// a "hold-off" period of 1s. During this time, we will buffer
		// any received messages, and publish them as soon as the period
		// is over. This prevents messsage loss on topics where this is
		// undesirable (e.g. H264 encoded video, where we would miss the
		// first keyframe packet).

		m_advertiseTime = ros::WallTime::now();
		m_inHoldoffTime = true;

		m_holdoffTimer = nh.createWallTimer(ros::WallDuration(1.0),
			std::bind(&Publisher::finishHoldoff, this)
		);

		m_advertised = true;
		m_advertisedMd5 = msg->md5;

		m_topicRewriterFuture = m_rewriter.open(msg->type, msg->md5);
	}

	// If the payload is empty, this is just an advertisement
	if(msg->payload.empty())
		return;

	if(m_inHoldoffTime)
		m_heldoffMsgs.push_back(msg);
	else
	{
		lock.unlock();
		internalPublish(msg);
	}
}

void Publisher::finishHoldoff()
{
	if(m_topicRewriterFuture.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
		return;

	m_topicRewriter = &m_topicRewriterFuture.get();

	std::unique_lock<std::mutex> lock(m_holdoffMutex);

	for(auto msg : m_heldoffMsgs)
		internalPublish(msg);
	m_heldoffMsgs.clear();

	m_inHoldoffTime = false;
	m_holdoffTimer.stop();
}

void Publisher::internalPublish(const Message::ConstPtr& msg)
{
	ROS_DEBUG("Publisher::internalPublish() for topic '%s'", msg->topic->name.c_str());

	if(msg->md5 != m_advertisedMd5)
	{
		ROS_ERROR_THROTTLE(1.0, "Received msg with md5 '%s' on topic '%s', but we already advertised with '%s'. Discarding...",
			msg->md5.c_str(),
			msg->topic->name.c_str(),
			m_advertisedMd5.c_str()
		);
		return;
	}

	boost::shared_ptr<topic_tools::ShapeShifter> shapeShifter(new topic_tools::ShapeShifter);
	shapeShifter->morph(msg->md5, msg->type, m_messageDefinition, "");

	auto rewritten = m_topicRewriter->rewrite(msg->payload);
	if(!rewritten.empty())
	{
		VectorStream stream{&rewritten};
		shapeShifter->read(stream);
	}
	else
	{
		VectorStream stream(&msg->payload);
		shapeShifter->read(stream);
	}

	m_pub.publish(shapeShifter);
}

}
