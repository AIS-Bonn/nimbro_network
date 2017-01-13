// De-fragments packets into complete messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_DEPACKETIZER_H
#define TT_DEPACKETIZER_H

#include <vector>
#include <list>
#include <cstdint>
#include <functional>

#include "../udp_packet.h"
#include "../message.h"

extern "C"
{
struct of_session;
struct of_parameters;
}

namespace nimbro_topic_transport
{

class Depacketizer
{
public:
	Depacketizer();

	typedef std::function<void(const std::string& topic, const Message::ConstPtr&)> Callback;

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

		std::shared_ptr<of_session> decoder;
		std::shared_ptr<of_parameters> params;
	};

	void pruneMessages();
	void handleMessagePacket(std::list<PartialMessage>::iterator it, const Packet::Ptr& packet);

	Callback m_cb;
	std::list<PartialMessage> m_messageBuffer;
};

}

#endif
