// Configuration and metadata for a single topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_TOPIC_H
#define TT_TOPIC_H

#include <string>
#include <memory>

#include <XmlRpcValue.h>

namespace nimbro_topic_transport
{

class Topic
{
public:
	typedef std::shared_ptr<Topic> Ptr;
	typedef std::shared_ptr<const Topic> ConstPtr;

	//! Topic name
	std::string name;

	//! All other configuration in XmlRpc format
	XmlRpc::XmlRpcValue config;
};

}

#endif
