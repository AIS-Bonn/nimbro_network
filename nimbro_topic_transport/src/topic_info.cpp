// Provides information about topic types
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_info.h"

#include <ros/names.h>

#include <mutex>

#include <boost/algorithm/string/trim.hpp>

#include "subprocess.h"

namespace nimbro_topic_transport
{

namespace topic_info
{

namespace
{
	std::string getHelperPath()
	{
		auto output = subprocess::obtainOutput("rosrun", {
			"rosrun", "--prefix", "echo", "nimbro_topic_transport", "get_msg_def.py"
		});

		if(!output)
		{
			ROS_ERROR("Could not find get_msg_def.py helper script");
			return {};
		}

		boost::algorithm::trim(*output);

		return *output;
	}

	std::string g_helperPath = getHelperPath();
}

/**
 * Execute a message query and return result.
 *
 * See utils/get_msg_def.py.
 *
 * @return output (message definition or md5sum)
 **/
static std::string msgQuery(const std::string& cmd, const std::string& type)
{
	if(g_helperPath.empty())
		throw std::runtime_error{"topic_info module was not properly initialized"};

	auto output = subprocess::obtainOutput(g_helperPath, {
		"get_msg_def.py", cmd, type
	});

	if(!output)
	{
		ROS_ERROR("Could not execute get_msg_def.py helper script for msg '%s'", type.c_str());
		return {};
	}

	return *output;
}

std::string getMsgDef(const std::string& type)
{
	return msgQuery("def", type);
}

std::string getMd5Sum(const std::string& type)
{
	return msgQuery("md5", type);
}

void packMD5(const std::string& str, LEValue< 4 >* dest)
{
	for(int i = 0; i < 4; ++i)
	{
		std::string md5_part = str.substr(8*i, 8);
		uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
		dest[i] = md5_num;
	}
}

void unpackMD5(const LEValue< 4 >* src, std::string* dest)
{
	dest->clear();
	for(int i = 0; i < 4; ++i)
	{
		char buf[10];
		snprintf(buf, sizeof(buf), "%08x", src[i]());
		*dest += buf;
	}
}

}

}
