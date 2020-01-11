// Rewrites all TF frame IDs
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "rewriter.h"

#include <boost/filesystem.hpp>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <ros/console.h>
#include <ros/package.h>

#include <dlfcn.h>
#include <sys/wait.h>

// Include gadget for types
#define MSG_INCLUDE <std_msgs/Header.h>
#define MSG_PACKAGE std_msgs
#define MSG_TYPE Header
#include "../../../data/gadget/rewrite_header.cpp"

namespace fs = boost::filesystem;

namespace nimbro_topic_transport
{

namespace
{
	class TopicRewriter
	{
	public:
		explicit TopicRewriter(const fs::path& library)
		{
			ROS_INFO("Loading TopicRewriter: %s\n", library.c_str());

			m_handle = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
			if(!m_handle)
			{
				ROS_ERROR("Could not open topic rewriter library '%s': %s",
					library.c_str(), dlerror()
				);
				ROS_ERROR("Maybe you should try to delete the above file.");
				return;
			}

			typedef MorphPtr (*RegisterFunc)();
			auto func = reinterpret_cast<RegisterFunc>(dlsym(m_handle, "registerMorph"));
			if(!func)
			{
				ROS_ERROR("Could not find register function in rewriter library %s: %s",
					library.c_str(), dlerror()
				);
				ROS_ERROR("Maybe you should try to delete the above file.");
				return;
			}

			m_morph = func();
		}

	private:
		void* m_handle = nullptr;
		MorphPtr m_morph = nullptr;
	};
}

class Rewriter::Private
{
public:
	std::unique_ptr<TopicRewriter> compile(const std::string& fullType, const std::string& md5)
	{
		auto split = fullType.find('/');
		if(split == std::string::npos)
		{
			ROS_ERROR("Could not split message type '%s' into parts", fullType.c_str());
			return {};
		}

		std::string package = fullType.substr(0, split);
		std::string type = fullType.substr(split+1);

		fs::path msgBinPath = binaryPath / (package + "___" + type + "___" + md5);
		fs::path outputFile = msgBinPath / "librewrite_header.so";
		fs::path inputFile = sourcePath / "data" / "gadget" / "rewrite_header.cpp";

		if(!fs::exists(outputFile) || fs::last_write_time(inputFile) > fs::last_write_time(outputFile))
		{
			if(!fs::exists(msgBinPath))
			{
				if(!fs::create_directories(msgBinPath))
				{
					ROS_ERROR("Could not create binary path '%s': %s", msgBinPath.c_str(), strerror(errno));
					return {};
				}
			}

			// Run compiler
			std::string pchFile = pchPath.string();

			std::vector<std::string> args{
				"g++",
				"-o", outputFile.string(),
				"-shared", "-std=c++17", "-O2", "-fPIC",
				"-include", pchFile,
				"-DMSG_INCLUDE=<" + fullType + ".h>",
				"-DMSG_PACKAGE=" + package,
				"-DMSG_TYPE=" + type
			};
			for(auto& a : includeFlags)
				args.push_back(a);
			args.push_back(inputFile.string());

			std::vector<char*> argsP;
			for(auto& a : args)
				argsP.push_back(a.data());
			argsP.push_back(nullptr);

			int pid = fork();
			if(pid == 0)
			{
				int ret = execvp("g++", argsP.data());
				if(ret != 0)
				{
					fprintf(stderr, "Could not execvp() g++: %s\n", strerror(errno));
				}

				std::abort();
			}
			else
			{
				int status;
				if(waitpid(pid, &status, 0) < 0)
				{
					ROS_ERROR("Could not waitpid(): %s", strerror(errno));
					return {};
				}

				if(!WIFEXITED(status) || WEXITSTATUS(status) != 0)
				{
					ROS_ERROR("Compilation of rewrite gadget for msg type %s failed (see stderr above)",
						fullType.c_str()
					);
					return {};
				}
			}
		}

		return std::make_unique<TopicRewriter>(outputFile);
	}

	std::string prefix;

	fs::path binaryPath;
	fs::path sourcePath;
	fs::path pchPath;

	std::vector<std::string> includeFlags;

// 	std::map<std::string, TopicRewriter> rewriters;
};

Rewriter::Rewriter(const std::string& prefix)
 : m_d{std::make_unique<Private>()}
{
	m_d->prefix = prefix;

	m_d->binaryPath = fs::path(getenv("HOME")) / ".ros" / "nimbro_topic_transport";
	m_d->sourcePath = fs::path(ros::package::getPath("nimbro_topic_transport"));
	if(m_d->sourcePath.empty())
		throw std::runtime_error("Could not find nimbro_topic_transport package");

	// Compile the precompiled header
	{
		fs::path pchDir = m_d->binaryPath / "pch";
		if(!fs::exists(pchDir))
		{
			if(!fs::create_directories(pchDir))
			{
				throw std::runtime_error("Could not create binary directory " + pchDir.string());
			}
		}

		const char* ROS_ROOT = getenv("ROS_ROOT");
		if(!ROS_ROOT)
			throw std::runtime_error("ROS_ROOT is not set, cannot find ROS includes");

		// Write ninja file
		{
			std::ofstream stream((pchDir / "build.ninja").string());
			if(!stream)
				throw std::runtime_error("Could not write ninja file to " + pchDir.string());

			stream << R"EOS(
cflags = -fPIC -std=c++17 -O2 -I)EOS" << ROS_ROOT << R"EOS(/../../include

rule cc_header
  deps = gcc
  depfile = $out.d
  command = g++ -x c++-header -MMD -MF $out.d $cflags -c -o $out $in

build ros_includes.h.gch: cc_header )EOS" << (m_d->sourcePath / "data" / "pch" / "ros_includes.h").string() << "\n";
		}

		// Execute ninja
		std::string buildDir = pchDir.string();

		int pid = fork();
		if(pid < 0)
		{
			throw std::runtime_error(std::string("Could not fork(): ") + strerror(errno));
		}

		if(pid == 0)
		{
			if(execlp("ninja", "ninja", "-C", buildDir.c_str(), static_cast<const char*>(nullptr)) != 0)
			{
				fprintf(stderr, "Could not execlp() ninja: %s\n", strerror(errno));
			}

			std::abort();
		}
		else
		{
			int status;
			if(waitpid(pid, &status, 0) < 0)
			{
				throw std::runtime_error(std::string("Could not waitpid(): ") + strerror(errno));
			}

			if(!WIFEXITED(status) || WEXITSTATUS(status) != 0)
			{
				throw std::runtime_error("ninja build failed. See above!");
			}
		}

		m_d->pchPath = pchDir / "ros_includes.h";
	}

	// Compute include dirs from CMAKE_PREFIX_PATH
	const char* CMAKE_PREFIX_PATH = getenv("CMAKE_PREFIX_PATH");
	if(!CMAKE_PREFIX_PATH)
	{
		throw std::runtime_error("CMAKE_PREFIX_PATH not defined");
	}

	std::vector<std::string> paths;
	boost::algorithm::split(paths, CMAKE_PREFIX_PATH, boost::algorithm::is_any_of(":"));

	for(const auto& path : paths)
	{
		m_d->includeFlags.push_back("-I" + path + "/include");
	}
}

Rewriter::~Rewriter() = default;

void Rewriter::prepare(const std::string& topicType, const std::string& md5)
{
	m_d->compile(topicType, md5);
}

boost::shared_array<uint8_t> Rewriter::rewrite(const std::string& topicType, const std::string& md5, const boost::shared_array<uint8_t>& msg)
{
	return {};
}

}
