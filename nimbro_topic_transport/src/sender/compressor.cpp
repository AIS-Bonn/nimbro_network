// Threaded compressor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "compressor.h"

#include <zstd.h>

namespace nimbro_topic_transport
{

//! Optimization: re-use source buffers for storing compressed results
static thread_local std::vector<uint8_t> g_compressionBuf;

unsigned int Compressor::getCompressionLevel(const Topic& topic)
{
	if(!topic.config.hasMember("compress"))
		return 0;

	// XmlRpc is not const-correct, so we need a copy here :-(
	auto config = topic.config;

	unsigned int level = 1; // Default ZSTD compression level (fast!)

	XmlRpc::XmlRpcValue compress = config["compress"];
	if(compress.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
	{
		if(!compress)
			return 0;
	}
	else if(compress.getType() == XmlRpc::XmlRpcValue::TypeInt)
	{
		level = (int)compress;
	}
	else
	{
		ROS_FATAL("Invalid 'compress' value on topic '%s'", topic.name.c_str());
		std::abort();
	}

	return level;
}

Compressor::Compressor(const Topic::ConstPtr&, unsigned int compressionLevel, Algorithm algorithm)
 : m_compressionLevel(compressionLevel)
 , m_algorithm(algorithm)
{
	if(m_algorithm == Algorithm::BZ2)
	{
		ROS_WARN(
			"BZ2 compression is obsolete, defaulting to ZSTD"
		);
	}
	m_algorithm = Algorithm::ZSTD;
}

Compressor::~Compressor()
{
}

Message::ConstPtr Compressor::compress(const Message::ConstPtr& msg)
{
	size_t len = ZSTD_compressBound(msg->payload.size());
	g_compressionBuf.resize(len);

	int ret = ZSTD_compress(
		g_compressionBuf.data(), len,              // dest
		msg->payload.data(), msg->payload.size(),  // source
		m_compressionLevel
	);

	if(ZSTD_isError(ret))
	{
		ROS_ERROR(
			"Could not compress data with ZSTD: '%s', sending uncompressed",
			ZSTD_getErrorName(ret)
		);

		return msg;
	}
	g_compressionBuf.resize(ret);

	// Create compressed message
	auto output = std::make_shared<Message>();
	output->payload.swap(g_compressionBuf);
	output->topic = msg->topic;
	output->type = msg->type;
	output->md5 = msg->md5;
	output->flags = msg->flags | Message::FLAG_COMPRESSED_ZSTD;

	return output;
}

}
