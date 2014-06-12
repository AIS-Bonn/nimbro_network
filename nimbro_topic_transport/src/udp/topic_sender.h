// Sends a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPIC_SENDER_H
#define TOPIC_SENDER_H

#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

namespace nimbro_topic_transport
{
	class UDPSender;

	class TopicSender
	{
	public:
		TopicSender(UDPSender* sender, ros::NodeHandle* nh, const std::string& topic, double rate, bool resend, int flags);
		~TopicSender();

		void handleData(const topic_tools::ShapeShifter::ConstPtr& shapeShifter);

		inline uint8_t* advance(int)
		{ return m_buf.data(); }
	private:
		void send(const topic_tools::ShapeShifter::ConstPtr& shapeShifter);
		void resend();

		UDPSender* m_sender;
		ros::Subscriber m_subscriber;
		std::string m_topicName;
		int m_flags;
		std::vector<uint8_t> m_buf;
		std::vector<uint8_t> m_compressionBuf;
		ros::Duration m_durationBetweenPackets;
		ros::Time m_lastTime;
		ros::Timer m_resendTimer;
		topic_tools::ShapeShifter::ConstPtr m_lastData;
		unsigned int m_msgCounter;
	};

};

#endif
