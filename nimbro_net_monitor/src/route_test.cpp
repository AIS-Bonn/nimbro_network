// Small test utility for route.c
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "route.h"

int main(int argc, char** argv)
{
	route::Cache cache;

	if(argc != 2)
	{
		fprintf(stderr, "Usage: route_test <host>\n");
		return 1;
	}

	std::string ifname = cache.obtainInterfaceForHost(argv[1]);

	printf("Host %s is reachable through interface %s\n",
		argv[1], ifname.c_str()
	);

	return 0;
}
