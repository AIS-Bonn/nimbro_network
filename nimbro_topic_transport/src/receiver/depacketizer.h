// De-fragments packets into complete messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_DEPACKETIZER_H
#define TT_DEPACKETIZER_H

#include <vector>
#include <list>
#include <cstdint>
#include <functional>
#include <mutex>

#include "../udp_packet.h"
#include "../message.h"

extern "C"
{
struct WirehairCodec_t;
}

namespace nimbro_topic_transport
{

class Depacketizer
{
public:
	Depacketizer();

	typedef std::function<void(const Message::ConstPtr&)> Callback;

	void setCallback(const Callback& cb);
	void addPacket(const Packet::Ptr& packet);
private:
	struct PartialMessage
	{
		explicit PartialMessage(uint32_t id)
		 : id(id)
		{}

		uint32_t id;
		bool complete = 0;
		std::size_t received_symbols = 0;
		std::vector<Packet::Ptr> packets;

		std::shared_ptr<WirehairCodec_t> decoder;
	};

	void pruneMessages();
	void handleMessagePacket(std::list<PartialMessage>::iterator it, const Packet::Ptr& packet);

	Callback m_cb;

	std::mutex m_mutex;
	std::list<PartialMessage> m_messageBuffer;
};

}

#endif
