// Publish received messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_RECEIVER_PUBLISHER_H
#define TT_RECEIVER_PUBLISHER_H

#include "../message.h"
#include "rewrite_headers/rewriter.h"

#include <mutex>

namespace nimbro_topic_transport
{

class Publisher
{
public:
	Publisher(const Topic::ConstPtr& topic, const std::string& prefix);
	~Publisher();

	void publish(const Message::ConstPtr& msg);
private:
	void internalPublish(const Message::ConstPtr& msg);
	void finishHoldoff();

	bool m_advertised = false;
	std::string m_prefix;

	bool m_inHoldoffTime = true;
	ros::WallTime m_advertiseTime;

	ros::WallTimer m_holdoffTimer;
	std::mutex m_holdoffMutex;
	std::vector<Message::ConstPtr> m_heldoffMsgs;

	ros::Publisher m_pub;

	std::string m_messageDefinition;

	Rewriter m_rewriter;
};

}

#endif
