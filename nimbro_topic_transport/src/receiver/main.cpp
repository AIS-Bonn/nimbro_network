// Simple main() for Receiver
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "receiver.h"

using namespace nimbro_topic_transport;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "receiver");

	ros::NodeHandle nh("~");

	Receiver receiver;

	ros::spin();

	return 0;
}
