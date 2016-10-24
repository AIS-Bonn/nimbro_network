// UDP sender node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <arpa/inet.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <ros/rate.h>
#include <ros/wall_timer.h>
#include <ros/publisher.h>

#include <deque>
#include <map>

#include <boost/thread.hpp>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

class TopicSender;

class UDPSender
{
public:
	UDPSender();
	~UDPSender();

	/**
	 * Allocate a new message ID. The message IDs are assigned sequentially
	 * and wrap around at 16 bits.
	 **/
	uint16_t allocateMessageID();

	/**
	 * Send data over the network.
	 *
	 * @note In relay mode, this actually just queues the message in the
	 *   named topic queue. In non-relay mode, the message is sent directly.
	 *
	 * @param data Data pointer
	 * @param size Size of the data buffer to be sent
	 * @param topic Topic name (to be included in the message header)
	 **/
	bool send(const void* data, uint32_t size, const std::string& topic);

	/**
	 * Some quirky WiFi routers have higher drop probabilities on the first
	 * packet in a burst - so here is a hacky parameter to duplicate those
	 * first parameters.
	 *
	 * This issue is best discovered using Wireshark with the provided filter
	 * scripts in utils/.
	 **/
	inline bool duplicateFirstPacket() const
	{ return m_duplicateFirstPacket; }

	//! Is FEC enabled?
	inline double fec() const
	{ return m_fec; }
private:
	/**
	 * Main thread for relay mode. See README.md and doc/configuration.md for
	 * details.
	 **/
	void relay();

	//! Actually send a message over the network
	bool internalSend(const void* data, uint32_t size, const std::string& topic);

	//! Update statistics (called from timer)
	void updateStats();

	uint16_t m_msgID; //!< current message ID

	//! @name Socket stuff
	//@{
	int m_fd;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;
	//@}

	ros::Time m_lastTime;
	bool m_duplicateFirstPacket;
	std::vector<TopicSender*> m_senders;

	//! @name Relay mode
	//@{
	bool m_relayMode;
	double m_relayRate;
	int m_relayTokensPerStep;
	uint64_t m_relayTokens;

	std::deque<std::vector<uint8_t>> m_relayBuffer;
	std::deque<std::string> m_relayNameBuffer;
	unsigned int m_relayIndex;

	boost::thread m_relayThread;
	bool m_relayThreadShouldExit;
	//@}

	double m_fec;

	//! @name Statistics
	//@{
	nimbro_topic_transport::SenderStats m_stats;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;
	uint64_t m_sentBytesInStatsInterval;

	// Count sent bytes for each topic seperately for logging
	std::map<std::string, uint64_t> m_sentTopicBytesInStatsInterval;
	//@}
};

}

#endif
