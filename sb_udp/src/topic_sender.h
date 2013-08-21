// Sends a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPIC_SENDER_H
#define TOPIC_SENDER_H

#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

namespace sb_udp
{
	class UDPSender;

	class TopicSender
	{
	public:
		TopicSender(UDPSender* sender, ros::NodeHandle* nh, const std::string& topic, double rate);

		void handleData(const topic_tools::ShapeShifter& shapeShifter);

		inline uint8_t* advance(int)
		{ return m_buf.data(); }
	private:
		UDPSender* m_sender;
		ros::Subscriber m_subscriber;
		std::string m_topicName;
		std::vector<uint8_t> m_buf;
		ros::Duration m_durationBetweenPackets;
		ros::Time m_lastTime;
	};

};

#endif
