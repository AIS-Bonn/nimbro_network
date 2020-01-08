// Provides information about topic types
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_info.h"

#include <ros/names.h>

#include <sys/wait.h>
#include <stdio.h>
#include <unistd.h>

#include <string>

namespace nimbro_topic_transport
{

namespace topic_info
{

/**
 * Execute a message query and return result.
 *
 * See utils/get_msg_def.py.
 *
 * @return output (message definition or md5sum)
 **/
static std::string msgQuery(const std::string& cmd, const std::string& type)
{
	std::vector<char> buf(1024);
	int idx = 0;

	std::string error;
	if(!ros::names::validate(type, error))
	{
		ROS_WARN("Got invalid message type '%s'", type.c_str());
		return "";
	}

	int fds[2];
	if(pipe(fds) != 0)
		throw std::runtime_error("Could not create pipe");

	int pid = fork();

	if(pid == 0)
	{
		close(fds[0]);
		dup2(fds[1], STDOUT_FILENO);

		if(execlp("rosrun", "rosrun", "nimbro_topic_transport", "get_msg_def.py", cmd.c_str(), type.c_str(), static_cast<char*>(nullptr)) != 0)
		{
			throw std::runtime_error("Could not execlp() rosmsg");
		}
	}

	close(fds[1]);

	while(1)
	{
		buf.resize(idx + 1024);
		int size = read(fds[0], buf.data() + idx, 1024);

		if(size < 0)
			throw std::runtime_error("Could not read()");
		else if(size == 0)
			break;

		idx += size;
	}

	close(fds[0]);

	int status;
	waitpid(pid, &status, 0);

	if(!WIFEXITED(status) || WEXITSTATUS(status) != 0 || idx == 0)
	{
		ROS_WARN("Could not get rosmsg %s for type '%s'", cmd.c_str(), type.c_str());
		return "";
	}

	return std::string(buf.data(), idx);
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
