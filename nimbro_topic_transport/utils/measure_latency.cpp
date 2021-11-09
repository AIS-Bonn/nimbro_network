// Receive msg from send_time and measure latency
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <std_msgs/Header.h>

void handleMsg(const ros::MessageEvent<std_msgs::Header>& msg)
{
	ros::Time now = ros::Time::now();
	ROS_INFO("Latency: %8.2fms (of which %3.2fms is internal ROS latency)",
		(now - msg.getMessage()->stamp).toSec() * 1000.0,
		(now - msg.getReceiptTime()).toSec() * 1000.0
	);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "measure_latency");

	if(argc != 2)
	{
		fprintf(stderr, "Usage: measure_latency [topic]\n");
		return 1;
	}

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(argv[1], 1, &handleMsg);

	ros::spin();

	return 0;
}
