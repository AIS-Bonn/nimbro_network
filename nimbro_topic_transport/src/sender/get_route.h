// Low-level network call: Find a network route
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_GET_ROUTE_H
#define TT_GET_ROUTE_H

#include <string>

#include <arpa/inet.h>

namespace nimbro_topic_transport
{

struct RouteInfo
{
	sockaddr_storage source_addr;
	socklen_t source_addr_len;
};

RouteInfo getRoute(const sockaddr* addr, socklen_t len);

}

#endif
