// Decompress compressed messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_DECOMPRESSOR_H
#define TT_DECOMPRESSOR_H

#include "../message.h"

namespace nimbro_topic_transport
{

class Decompressor
{
public:
	Decompressor();
	~Decompressor();

	Message::ConstPtr decompress(const Message::ConstPtr& msg);
private:
	std::vector<uint8_t> m_buf;
};

}

#endif
