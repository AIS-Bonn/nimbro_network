// Message compression
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_COMPRESSOR_H
#define TT_COMPRESSOR_H

#include "message.h"

#include <thread>
#include <mutex>
#include <condition_variable>

namespace nimbro_topic_transport
{

class Compressor
{
public:
	typedef std::function<void(const Message::ConstPtr& msg)> Callback;

	explicit Compressor(const Topic::ConstPtr& topic, unsigned int compressionLevel);
	~Compressor();

	//! Called from input (usually Subscriber)
	void handleInput(const Message::ConstPtr& msg);

	void registerCallback(const Callback& cb);

	static unsigned int getCompressionLevel(const Topic& topic);
private:
	std::vector<Callback> m_callbacks;
	unsigned int m_compressionLevel = 1;
};

}

#endif
