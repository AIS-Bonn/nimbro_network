// Sends time (used for latency measurements)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <std_msgs/Header.h>

#include <cstdio>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "send_time");

	if(argc != 2)
	{
		fprintf(stderr, "Usage: send_time <rate>\n");
		return 1;
	}

	ros::NodeHandle nh{};

	ros::Publisher pub = nh.advertise<std_msgs::Header>("time", 1);

	ros::Rate rate(atoi(argv[1]));

	while(ros::ok())
	{
		std_msgs::Header msg;
		msg.stamp = ros::Time::now();
		pub.publish(msg);
		rate.sleep();
	}

	return 0;
}
