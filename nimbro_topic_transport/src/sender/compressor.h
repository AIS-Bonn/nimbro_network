// Message compression
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_COMPRESSOR_H
#define TT_COMPRESSOR_H

#include "topic.h"
#include "../message.h"

#include <thread>
#include <mutex>
#include <condition_variable>

namespace nimbro_topic_transport
{

class Compressor
{
public:
	enum class Algorithm
	{
		ZSTD,
		BZ2
	};

	explicit Compressor(
		const Topic::ConstPtr& topic,
		unsigned int compressionLevel = 1,
		Algorithm algorithm = Algorithm::ZSTD
	);
	~Compressor();

	Message::ConstPtr compress(const Message::ConstPtr& msg);

	static unsigned int getCompressionLevel(const Topic& topic);
private:
	unsigned int m_compressionLevel = 1;
	Algorithm m_algorithm = Algorithm::ZSTD;
};

}

#endif
