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

Publisher::Publisher(const Topic::ConstPtr& topic, const std::string& prefix)
 : m_prefix{prefix}
{
}

Publisher::~Publisher()
{
}

void Publisher::publish(const Message::ConstPtr& msg)
{
	std::unique_lock<std::mutex> lock(m_holdoffMutex);

	if(!m_advertised)
	{
		ROS_INFO("Advertising new topic '%s'", msg->topic->name.c_str());
		m_messageDefinition = topic_info::getMsgDef(msg->type);

		ros::NodeHandle nh;

		ros::AdvertiseOptions options(
			m_prefix + msg->topic->name,
			50,
			msg->md5,
			msg->type,
			m_messageDefinition
		);

		m_pub = nh.advertise(options);

		// Because subscribers will not immediately subscribe, we introduce
		// a "hold-off" period of 500ms. During this time, we will buffer
		// any received messages, and publish them as soon as the period
		// is over. This prevents messsage loss on topics where this is
		// undesirable (e.g. H264 encoded video, where we would miss the
		// first keyframe packet).

		m_advertiseTime = ros::WallTime::now();
		m_inHoldoffTime = true;

		m_holdoffTimer = nh.createWallTimer(ros::WallDuration(0.5),
			std::bind(&Publisher::finishHoldoff, this), true
		);

		m_advertised = true;
	}

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
	std::unique_lock<std::mutex> lock(m_holdoffMutex);

	for(auto msg : m_heldoffMsgs)
		internalPublish(msg);
	m_heldoffMsgs.clear();

	m_inHoldoffTime = false;
}

void Publisher::internalPublish(const Message::ConstPtr& msg)
{
	// FIXME: Check type again here?

	boost::shared_ptr<topic_tools::ShapeShifter> shapeShifter(new topic_tools::ShapeShifter);
	shapeShifter->morph(msg->md5, msg->type, m_messageDefinition, "");

	VectorStream stream(&msg->payload);
	shapeShifter->read(stream);

	m_pub.publish(shapeShifter);
}

}
