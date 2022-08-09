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

	Publisher(const Publisher&) = delete;
	Publisher& operator=(const Publisher&) = delete;

	void publish(const Message::ConstPtr& msg);
private:
	void internalPublish(const Message::ConstPtr& msg);
	void finishHoldoff();
	void printStats();

	Topic::ConstPtr m_topic;

	std::mutex m_messageIDMutex;
	uint16_t m_lastMessageID = 0;
	uint32_t m_seed = 0;

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

	ros::Timer m_statsTimer;
	std::mutex m_statsMutex;
	uint64_t m_statCounter = 0;
	ros::Duration m_statDelay;
};

}

#endif
