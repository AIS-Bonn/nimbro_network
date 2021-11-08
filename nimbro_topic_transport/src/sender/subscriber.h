// Start of the topic transport pipeline.
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_SUBSCRIBER_H
#define TT_SUBSCRIBER_H

#include "../message.h"

#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <unordered_set>

namespace nimbro_topic_transport
{

class Subscriber
{
public:
	typedef std::function<void(const Message::ConstPtr&)> Callback;

	Subscriber(const Topic::Ptr& topic, ros::NodeHandle& nh, const std::string& fullTopicName);

	void registerCallback(const Callback& cb);
	void sendAdvertisement(const std::string& typeHint = {});

	std::string rosTopicName() const
	{ return m_subscriber.getTopic(); }
private:
	void handleData(const ros::MessageEvent<topic_tools::ShapeShifter>& event);

	Topic::Ptr m_topic;
	ros::Subscriber m_subscriber;

	std::vector<Callback> m_callbacks;

	//! @name Rate control logic
	//@{
	void resend();

	ros::Duration m_durationBetweenMsgs = ros::Duration(0.0);
	ros::Time m_lastTime;
	ros::Timer m_resendTimer;
	Message::ConstPtr m_lastMsg;
	//@}

	std::string m_type;
	std::string m_md5;
	uint32_t m_counter = 0;

	std::unordered_set<std::string> m_excludedPublishers;

	//! @name Statistics
	//@{
	void printStats();

	uint64_t m_incomingMessages = 0;
	uint64_t m_filteredMessages = 0;
	ros::SteadyTimer m_statTimer;
	//@}
};

}

#endif
