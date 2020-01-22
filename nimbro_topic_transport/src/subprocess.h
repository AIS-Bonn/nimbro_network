// Execute & monitor a sub process
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef TT_SUBPROCESS_H
#define TT_SUBPROCESS_H

#include <string>
#include <optional>
#include <vector>

namespace nimbro_topic_transport
{
namespace subprocess
{

bool call(const std::string& program, const std::vector<std::string>& args);

std::optional<std::string> obtainOutput(const std::string& program, const std::vector<std::string>& args);

}
}

#endif
