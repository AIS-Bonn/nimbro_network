// Common methods for TCP & UDP transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMMON_H
#define COMMON_H

#include <string>

namespace nimbro_service_transport
{

std::string getServiceMD5(const std::string& type);

}

#endif
