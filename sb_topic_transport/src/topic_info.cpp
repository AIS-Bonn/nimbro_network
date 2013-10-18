// Provides information about topic types
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_info.h"

#include <ros/names.h>
#include <stdio.h>

namespace sb_topic_transport
{

namespace topic_info
{

std::string getMsgDef(const std::string& type)
{
	std::vector<char> buf(1024);
	int idx = 0;

	std::string error;
	if(!ros::names::validate(type, error))
	{
		ROS_WARN("Got invalid message type '%s'", type.c_str());
		return "";
	}

	// FIXME: This is fricking dangerous!
	FILE* f = popen(("rosmsg show \'" + type + "\'").c_str(), "r");

	while(!feof(f))
	{
		buf.resize(idx + 1024);
		size_t size = fread(buf.data() + idx, 1, 1024, f);
		if(size == 0)
			break;

		idx += size;
	}

	int exit_code = pclose(f);

	if(exit_code != 0)
	{
		ROS_WARN("Could not get msg def for type '%s'", type.c_str());
		return "";
	}
	else
	{
		return std::string(buf.data(), idx);
	}
}

std::string getMd5Sum(const std::string& type)
{
	std::vector<char> buf(1024);
	int idx = 0;

	std::string error;
	if(!ros::names::validate(type, error))
	{
		ROS_WARN("Got invalid message type '%s'", type.c_str());
		return "";
	}

	// FIXME: This is fricking dangerous!
	FILE* f = popen(("rosmsg md5 \'" + type + "\'").c_str(), "r");

	while(!feof(f))
	{
		buf.resize(idx + 1024);
		size_t size = fread(buf.data() + idx, 1, 1024, f);
		if(size == 0)
			break;

		idx += size;
	}

	int exit_code = pclose(f);

	if(exit_code != 0)
	{
		fprintf(stderr, "Could not get md5 sum for type '%s'\n", type.c_str());
		return "";
	}
	else
	{
		std::string ret(buf.data(), idx-1);
		return ret;
	}
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