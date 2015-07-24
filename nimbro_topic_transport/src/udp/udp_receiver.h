// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <map>
#include <vector>
#include <string>

#include <ros/publisher.h>

#include <topic_tools/shape_shifter.h>

#include <list>

#include <boost/thread.hpp>

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
	 , decoder(0)
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
	std::vector<std::vector<uint8_t>> fecPackets;
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

	void takeForDecompression(const boost::shared_ptr<Message>& compressed);
private:
	boost::shared_ptr<Message> m_compressedMsg;
	boost::condition_variable m_cond;
	boost::mutex m_mutex;

	boost::thread m_decompressionThread;
	bool m_decompressionThreadRunning;
	bool m_decompressionThreadShouldExit;

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
};

}

#endif
