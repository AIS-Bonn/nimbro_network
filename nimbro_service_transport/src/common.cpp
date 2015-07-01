// Common methods for TCP & UDP transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "common.h"

#include <vector>

#include <ros/package.h>
#include <ros/names.h>

namespace nimbro_service_transport
{

std::string getServiceMD5(const std::string& type)
{
	std::vector<char> buf(1024);
	int idx = 0;

	std::string error;
	if(!ros::names::validate(type, error))
	{
		ROS_WARN("Got invalid service type '%s'", type.c_str());
		return "";
	}

	// FIXME: This is fricking dangerous!
	FILE* f = popen(
		std::string(ros::package::getPath("nimbro_service_transport") + "/scripts/get_md5.py \'" + type + "\'").c_str(), "r"
	);

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
		ROS_ERROR("Could not get md5 sum for service type '%s'", type.c_str());
		return "*";
	}
	else
	{
		return std::string(buf.data(), idx);
	}
}

}

