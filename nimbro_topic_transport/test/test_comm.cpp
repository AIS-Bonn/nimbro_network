// Unit tests for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <catch_ros/catch.hpp>

#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/init.h>

#include <std_msgs/Int64.h>
#include <std_msgs/UInt64MultiArray.h>

int g_counter = 0;

void handleMessageLocal(const std_msgs::Int64 &msg)
{
	REQUIRE(g_counter == msg.data);
	g_counter++;
}

int g_arrayCounter = 0;

void handle_array(const std_msgs::UInt64MultiArray& msg)
{
	REQUIRE(msg.data.size() == 512);
	for(int i = 0; i < 512; ++i)
	{
		REQUIRE(msg.data[i] == i);
	}
	g_arrayCounter++;
}

const int HUGE_SIZE = 3 * 1024;

void handle_huge(const std_msgs::UInt64MultiArray& msg)
{
	REQUIRE(msg.data.size() == HUGE_SIZE);
	for(int i = 0; i < HUGE_SIZE; ++i)
	{
		REQUIRE(msg.data[i] == i);
	}
	g_arrayCounter++;
}

TEST_CASE("simple", "[topic]")
{
	std_msgs::Int64 msg;

	g_counter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::Int64>("/test_topic", 2);
	ros::Subscriber sub = nh.subscribe("/receive/test_topic", 2, &handleMessageLocal);

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

	msg.data = 0;
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	msg.data = 1;
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(g_counter == 2)
			return;
	}

	FAIL();
}

TEST_CASE("array", "[topic]")
{
	std_msgs::UInt64MultiArray msg;

	g_arrayCounter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::UInt64MultiArray>("/array_topic", 2);
	ros::Subscriber sub;

	SECTION("small")
	{
		sub = nh.subscribe("/receive/array_topic", 2, &handle_array);
		for(int i = 0; i < 512; ++i)
			msg.data.push_back(i);
	}
	SECTION("huge")
	{
		sub = nh.subscribe("/receive/array_topic", 2, &handle_huge);
		for(int i = 0; i < HUGE_SIZE; ++i)
			msg.data.push_back(i);
	}

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

	g_arrayCounter = 0;
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(g_arrayCounter == 2)
			return;
	}

	CAPTURE(g_arrayCounter);
	FAIL();
}
