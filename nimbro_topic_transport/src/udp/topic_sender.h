// Sends a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPIC_SENDER_H
#define TOPIC_SENDER_H

#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <boost/thread.hpp>

#if WITH_CONFIG_SERVER
#include <config_server/parameter.h>
#endif

namespace nimbro_topic_transport
{
	class UDPSender;

	class TopicSender
	{
	public:
		TopicSender(UDPSender* sender, ros::NodeHandle* nh, const std::string& topic, double rate, bool resend, int flags, bool enable = true, const std::string& type = "");
		~TopicSender();

		void handleData(const topic_tools::ShapeShifter::ConstPtr& shapeShifter);

		inline uint8_t* advance(int)
		{ return m_buf.data(); }

		bool isDirectTransmissionEnabled() const
		{ return m_directTransmission; }
		void setDirectTransmissionEnabled(bool value);

		void sendCurrentMessage();
	private:
		void send();
		void resend();

		void sendWithFEC();
		void sendWithoutFEC();

		static std::string escapeTopicName(std::string topicName);

		UDPSender* m_sender;
		ros::Subscriber m_subscriber;
		std::string m_topicName;
		std::string m_topicType;
		int m_flags;
		std::vector<uint8_t> m_buf;
		uint32_t m_md5[4];

		std::vector<uint8_t> m_compressionBuf;
		ros::Duration m_durationBetweenPackets;
		ros::Time m_lastTime;
		ros::Timer m_resendTimer;
		topic_tools::ShapeShifter::ConstPtr m_lastData;
		bool m_updateBuf;
		unsigned int m_msgCounter;
		unsigned int m_inputMsgCounter;

		bool m_directTransmission;

		boost::mutex m_dataMutex;

#if WITH_CONFIG_SERVER
		config_server::Parameter<bool> m_enable;
#endif
	};

};

#endif
