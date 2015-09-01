// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <map>
#include <vector>
#include <string>
#include <list>

#include <sys/socket.h>

#include <ros/publisher.h>
#include <ros/wall_timer.h>

#include <topic_tools/shape_shifter.h>

#include <boost/thread.hpp>

#include <nimbro_topic_transport/ReceiverStats.h>
#include <nimbro_topic_transport/CompressedMsg.h>

#include "udp_packet.h"

#if WITH_OPENFEC
extern "C"
{
#include <of_openfec_api.h>
}
#endif

namespace nimbro_topic_transport
{

struct Message
{
	Message(uint16_t id)
	 : id(id)
	 , size(0)
	 , complete(false)
	{}

	Message()
	{}

	~Message()
	{
	}

	uint32_t getLength() const
	{ return size; }

	uint8_t* getData()
	{ return payload.data(); }

	bool decompress(Message* dest);

	uint16_t id;
	UDPFirstPacket::Header header;
	std::vector<uint8_t> payload;
	size_t size;
	std::vector<bool> msgs;

	bool complete;

#if WITH_OPENFEC
	boost::shared_ptr<of_session_t> decoder;
	boost::shared_ptr<of_parameters_t> params;
	unsigned int received_symbols;
	std::vector<boost::shared_ptr<std::vector<uint8_t>>> fecPackets;
#endif
};

struct TopicData
{
	TopicData();
	~TopicData();

	ros::Publisher publisher;

	uint32_t md5[4];
	std::string md5_str;
	std::string msg_def;

	bool compressed;

	int last_message_counter;

	bool waiting_for_subscriber;

	void takeForDecompression(const boost::shared_ptr<Message>& compressed);

	void publish(const boost::shared_ptr<topic_tools::ShapeShifter>& msg);
	void publishCompressed(const CompressedMsgConstPtr& msg);
	void handleSubscriber();
private:
	boost::shared_ptr<Message> m_compressedMsg;
	boost::condition_variable m_cond;
	boost::mutex m_mutex;

	boost::thread m_decompressionThread;
	bool m_decompressionThreadRunning;
	bool m_decompressionThreadShouldExit;

	boost::shared_ptr<topic_tools::ShapeShifter> m_msgInQueue;
	CompressedMsgConstPtr m_compressedMsgInQueue;
	ros::WallTime m_queueTime;

	void decompress();
};

class UDPReceiver
{
public:
	UDPReceiver();
	~UDPReceiver();

	void run();
private:
	typedef std::map<std::string, boost::shared_ptr<TopicData>> TopicMap;
	typedef std::list<Message> MessageBuffer;

	template<class HeaderType>
	void handleFinishedMessage(Message* msg, HeaderType* header);

	void updateStats();

	int m_fd;
	MessageBuffer m_incompleteMessages;
	TopicMap m_topics;

	bool m_dropRepeatedMessages;
	bool m_warnDropIncomplete;
	bool m_keepCompressed;

	ros::NodeHandle m_nh;
	ros::Publisher m_pub_heartbeat;
	ros::Time m_lastHeartbeatTime;

	ros::Publisher m_pub_plot;

	Message m_decompressedMessage;

	bool m_fec;

	nimbro_topic_transport::ReceiverStats m_stats;
	uint64_t m_receivedBytesInStatsInterval;
	uint64_t m_expectedPacketsInStatsInterval;
	uint64_t m_missingPacketsInStatsInterval;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;

	sockaddr_storage m_remoteAddr;
	socklen_t m_remoteAddrLen;
};

}

#endif
