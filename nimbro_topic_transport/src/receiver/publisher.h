// Publish received messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_RECEIVER_PUBLISHER_H
#define TT_RECEIVER_PUBLISHER_H

#include "../message.h"
#include "rewriter.h"

#include <mutex>

namespace nimbro_topic_transport
{

class Publisher
{
public:
	Publisher(const Topic::ConstPtr& topic, Rewriter& rewriter);
	~Publisher();

	void publish(const Message::ConstPtr& msg);
private:
	void internalPublish(const Message::ConstPtr& msg);
	void finishHoldoff();

	bool m_advertised = false;

	bool m_inHoldoffTime = true;
	ros::WallTime m_advertiseTime;

	ros::WallTimer m_holdoffTimer;
	std::mutex m_holdoffMutex;
	std::vector<Message::ConstPtr> m_heldoffMsgs;

	ros::Publisher m_pub;

	std::string m_messageDefinition;

	Rewriter& m_rewriter;
	Rewriter::TopicRewriterFuture m_topicRewriterFuture;
	const Rewriter::TopicRewriter* m_topicRewriter = nullptr;

	std::string m_advertisedMd5;
};

}

#endif
