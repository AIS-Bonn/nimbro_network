// Integrated transceiver
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "sender/sender.h"
#include "receiver/receiver.h"

using namespace nimbro_topic_transport;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transceiver");

	ros::NodeHandle nh{"~"};

	Sender sender{ros::NodeHandle{"~/sender"}};
	Receiver receiver{ros::NodeHandle{"~/receiver"}};

	ros::spin();

	return 0;
}
