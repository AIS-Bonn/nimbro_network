// Low-level network call: Find a network route
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "get_route.h"

#include <unistd.h>

#include <netdb.h>

namespace nimbro_topic_transport
{

RouteInfo getRoute(const sockaddr* addr, socklen_t len)
{
	int fd = socket(addr->sa_family, SOCK_DGRAM, 0);
	if(fd < 0)
	{
		perror("Could not create socket");
		return RouteInfo();
	}

	if(connect(fd, addr, len) != 0)
	{
		perror("Could not connect socket");
		return RouteInfo();
	}

	RouteInfo out;
	out.source_addr_len = sizeof(out.source_addr);

	if(getsockname(fd, (sockaddr*)&out.source_addr, &out.source_addr_len) != 0)
	{
		perror("Could not get local address");
		return RouteInfo();
	}

	close(fd);

	return out;
}

}

#if 0
int main(int argc, char** argv)
{
	using namespace nimbro_topic_transport;

	addrinfo* info = 0;

	getaddrinfo(argv[1], 0, 0, &info);

	auto route = getRoute(info->ai_addr, info->ai_addrlen);

	char buf[256];
	getnameinfo((sockaddr*)&route.source_addr, route.source_addr_len, buf, sizeof(buf), 0, 0, NI_NUMERICHOST);

	printf("Got source address: %s\n", buf);
	return 0;
}
#endif
