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
		/**
		 * Constructor.
		 *
		 * @param sender Parent UDPSender instance
		 * @param nh ROS node handle for topic subscription
		 * @param topic Topic name
		 * @param rate Maximum message rate
		 * @param resend If true, @a rate specifies the minimum rate as well.
		 *   Messages are repeated if needed.
		 * @param flags Other flags (see udp_packet.h)
		 * @param enable Default enable status
		 *   (only used if nimbro_config_server is available)
		 * @param type Initialize the msg type of the subscription. If empty,
		 *   we will use a wildcard subscription.
		 **/
		TopicSender(
			UDPSender* sender, ros::NodeHandle* nh,
			const std::string& topic, double rate,
			bool resend, int flags, bool enable = true,
			const std::string& type = ""
		);
		~TopicSender();

		//! Data callback
		void handleData(const topic_tools::ShapeShifter::ConstPtr& shapeShifter);

		/**
		 * Control direct transmission.
		 *
		 * @param value If true, incoming messages from the ROS topic are immediately
		 *   sent via the UDPSender::send() method. If false, you will need to call
		 *   sendCurrentMessage() to trigger sending.
		 **/
		void setDirectTransmissionEnabled(bool value);
		bool isDirectTransmissionEnabled() const
		{ return m_directTransmission; }

		enum CompressionType
		{
			COMPRESSION_NONE,
			COMPRESSION_BZ2,
			COMPRESSION_ZSTD,
		};

		void setCompression(CompressionType type, int level);

		/**
		 * Send latest message.
		 *
		 * @sa setDirectTransmissionEnabled()
		 **/
		void sendCurrentMessage();
	private:
		class VectorBuffer : public std::vector<uint8_t>
		{
		public:
			void beginWrite()
			{
				m_offset = 0;
			}

			//! ShapeShifter write interface
			inline uint8_t* advance(int off)
			{
				uint8_t* ptr = data() + m_offset;
				m_offset += off;
				return ptr;
			}
		private:
			std::size_t m_offset = 0;
		};

		//! Send m_lastData
		void send();

		//! Resend timer, used if resend = true
		void resend();

		void sendWithFEC();
		void sendWithoutFEC();

		//! Remove all "/" from topic name
		static std::string escapeTopicName(std::string topicName);

		UDPSender* m_sender;
		ros::Subscriber m_subscriber;
		std::string m_topicName;
		std::string m_topicType;
		int m_flags;
		VectorBuffer m_buf;
		int m_bufFlags;
		uint32_t m_md5[4];

		std::vector<uint8_t> m_compressionBuf;
		ros::Duration m_durationBetweenPackets;
		ros::Time m_lastTime;
		ros::Timer m_resendTimer;
		topic_tools::ShapeShifter::ConstPtr m_lastData; // protected by m_dataMutex
		bool m_updateBuf;
		unsigned int m_msgCounter;
		unsigned int m_inputMsgCounter;

		bool m_directTransmission;

		boost::mutex m_dataMutex; // Protects m_lastData

#if WITH_CONFIG_SERVER
		config_server::Parameter<bool> m_enable;
#endif

		CompressionType m_compression;
		int m_compressionLevel;
	};

}

#endif
