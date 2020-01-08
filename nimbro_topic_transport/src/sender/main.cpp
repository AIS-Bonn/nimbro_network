// Simple main() for Sender
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "sender.h"

using namespace nimbro_topic_transport;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sender");

	Sender sender;

	ros::spin();

	return 0;
}
