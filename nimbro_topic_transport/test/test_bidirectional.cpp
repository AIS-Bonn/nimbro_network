// Unit tests for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <catch_ros/catch.hpp>

#include <ros/package.h>
#include <ros/param.h>
#include <ros/node_handle.h>

#include <std_msgs/Int64.h>

int g_counter = 0;

void handleMessageLocal(const std_msgs::Int64 &msg)
{
	//REQUIRE(g_counter == msg.data);
	g_counter++;
}

void
testOneDirection(ros::NodeHandle &nh, bool allow_bidirectional, std::string source_topic, std::string sink_topic) {

	ros::Publisher pub = nh.advertise<std_msgs::Int64>(source_topic, 2);
	ros::Subscriber sub = nh.subscribe(sink_topic, 2, &handleMessageLocal);

	int timeout = 50;
	while(pub.getNumSubscribers() == 0)
	{
		ros::spinOnce();
		if(--timeout == 0)
		{
			CAPTURE(pub.getNumSubscribers());
			FAIL("sender did not subscribe on our topic");
			return;
		}

		usleep(20 * 1000);
	}

	g_counter = 0;

	std_msgs::Int64 msg;
	msg.data = 0;
	pub.publish(msg);

	ROS_INFO("Sent first message, waiting for 5 seconds to receive it.");

	for(int i = 0; i < 5000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	msg.data = 1;
	pub.publish(msg);

	ROS_INFO("Sent second message, waiting for 5 seconds to receive it.");

	for(int i = 0; i < 5000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	if (allow_bidirectional) {
		if (g_counter == 2) {
			ROS_INFO("With bidirectional support, each message was received "
				"exactly once.");
			return;
		}
	} else {
		if (g_counter > 10) {
			ROS_INFO_STREAM("Without bidirectional support, the relay entered "
				"an infinite loop; the messsages were received "
				<< g_counter << " times.");
			CAPTURE(g_counter);
			return;
		}
	}
	CAPTURE(allow_bidirectional);
	CAPTURE(g_counter);
	FAIL();
}

TEST_CASE("bidirectional_communication", "[topic]")
{
	ros::NodeHandle nh("~");
	bool allow_bidirectional;
	nh.getParam("allow_bidirectional", allow_bidirectional);

	ROS_INFO("Testing machine1 to machine2 direction.");
	testOneDirection(nh, allow_bidirectional, "/my_first_topic", "/recv/my_first_topic");

	ROS_INFO("Testing machine2 to machine1 direction.");
	testOneDirection(nh, allow_bidirectional, "/recv/my_first_topic", "/my_first_topic");
}