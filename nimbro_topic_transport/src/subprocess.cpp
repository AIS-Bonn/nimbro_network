// Execute & monitor a sub process
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "subprocess.h"

#include <sys/types.h>
#include <sys/wait.h>

#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <ros/console.h>

namespace nimbro_topic_transport
{
namespace subprocess
{

bool call(const std::string& program, const std::vector<std::string>& args)
{
	// removing const is ok - args are only modified in the child process.
	std::vector<char*> argsP;
	for(auto& a : args)
		argsP.push_back(const_cast<char*>(a.data()));
	argsP.push_back(nullptr);

	int pid = fork();
	if(pid == 0)
	{
		// Child
		int ret = execvp(program.c_str(), argsP.data());
		if(ret != 0)
		{
			fprintf(stderr, "Could not execvp() g++: %s\n", strerror(errno));
		}

		std::abort();
	}

	// Parent
	int status;
	if(waitpid(pid, &status, 0) < 0)
	{
		ROS_ERROR("Could not waitpid(): %s", strerror(errno));
		return false;
	}

	if(!WIFEXITED(status) || WEXITSTATUS(status) != 0)
		return false;

	return true;
}

std::optional<std::string> obtainOutput(const std::string& program, const std::vector<std::string>& args)
{
	// removing const is ok - args are only modified in the child process.
	std::vector<char*> argsP;
	for(auto& a : args)
		argsP.push_back(const_cast<char*>(a.data()));
	argsP.push_back(nullptr);

	int fds[2];
	if(pipe(fds) != 0)
	{
		ROS_ERROR("Could not create pipe: %s", strerror(errno));
		return {};
	}

	int pid = fork();
	if(pid == 0)
	{
		// Child
		close(fds[0]);
		dup2(fds[1], STDOUT_FILENO);

		int ret = execvp(program.c_str(), argsP.data());
		if(ret != 0)
		{
			fprintf(stderr, "Could not execvp() g++: %s\n", strerror(errno));
		}

		std::abort();
	}

	// Parent
	close(fds[1]);

	std::vector<char> buf;
	std::size_t idx = 0;
	while(1)
	{
		buf.resize(idx + 1024);
		int size = read(fds[0], buf.data() + idx, 1024);

		if(size < 0)
		{
			ROS_ERROR("Could not read() from child process: %s", strerror(errno));
			close(fds[0]);
			return {};
		}
		else if(size == 0)
			break;

		idx += size;
	}

	close(fds[0]);

	int status;
	if(waitpid(pid, &status, 0) < 0)
	{
		ROS_ERROR("Could not waitpid(): %s", strerror(errno));
		return {};
	}

	if(!WIFEXITED(status) || WEXITSTATUS(status) != 0)
		return {};

	return std::string{buf.data(), idx};
}

}
}
