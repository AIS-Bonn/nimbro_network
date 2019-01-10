// Obtain IP routes from the kernel
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NET_MONITOR_ROUTE_H
#define NET_MONITOR_ROUTE_H

#include <string>
#include <memory>
#include <map>

struct mnl_socket;

namespace route
{

class Cache
{
public:
	Cache();
	~Cache();

	std::string obtainInterfaceForHost(const std::string& host);
private:
	std::shared_ptr<mnl_socket> m_mnl;
	std::map<std::string, std::string> m_interfaceCache;
};

}

#endif
