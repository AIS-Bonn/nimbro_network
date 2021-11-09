// Receive msg from tf and measure latency
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>

void handleMsg(const ros::MessageEvent<tf2_msgs::TFMessage>& msg)
{
	auto tfMsg = msg.getMessage();
	if(tfMsg->transforms.empty())
		return;

	ros::Time now = ros::Time::now();
	ROS_INFO("Latency: %8.2fms (of which %3.2fms is internal ROS latency)",
		(now - tfMsg->transforms[0].header.stamp).toSec() * 1000.0,
		(now - msg.getReceiptTime()).toSec() * 1000.0
	);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "measure_tf_latency", ros::init_options::AnonymousName);

	if(argc != 2)
	{
		fprintf(stderr, "Usage: measure_tf_latency [topic]\n");
		return 1;
	}

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(argv[1], 1, &handleMsg);

	ros::spin();

	return 0;
}

