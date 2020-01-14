// Unit tests for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <catch_ros/catch.hpp>

#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/init.h>

#include <std_msgs/Int64.h>
#include <std_msgs/UInt64MultiArray.h>

#include <geometry_msgs/TransformStamped.h>

int g_counter = 0;

void handle_simple(const std_msgs::Int64& msg)
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

	ros::Publisher pub;
	ros::Subscriber sub;

	SECTION("simple")
	{
		pub = nh.advertise<std_msgs::Int64>("test_topic", 2);
		sub = nh.subscribe("/receive/" + nh.resolveName("test_topic"), 2, &handle_simple);
	}

	SECTION("remove prefix")
	{
		pub = nh.advertise<std_msgs::Int64>("/odd_prefix/" + nh.resolveName("unprefix_me"), 2);
		sub = nh.subscribe("/receive/" + nh.resolveName("unprefix_me"), 2, &handle_simple);
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

	g_counter = 0;

	msg.data = 0;
	pub.publish(msg);

	for(int i = 0; i < 100; ++i)
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

	CAPTURE(g_counter);
	FAIL("Not enough messages received on topic " << sub.getTopic());
}

TEST_CASE("array", "[topic]")
{
	std_msgs::UInt64MultiArray msg;

	g_arrayCounter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::UInt64MultiArray>("array_topic", 2);
	ros::Subscriber sub;

	SECTION("small")
	{
		sub = nh.subscribe("/receive/" + nh.resolveName("array_topic"), 2, &handle_array);
		for(int i = 0; i < 512; ++i)
			msg.data.push_back(i);
	}
	SECTION("huge")
	{
		sub = nh.subscribe("/receive/" + nh.resolveName("array_topic"), 2, &handle_huge);
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

	usleep(50 * 1000);

	g_arrayCounter = 0;
	pub.publish(msg);

	for(int i = 0; i < 100; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	INFO("Received after first message was sent: " << g_arrayCounter);
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(g_arrayCounter == 2)
			return;
	}

	CAPTURE(g_arrayCounter);
	FAIL("Not enough messages received");
}

TEST_CASE("rewriting", "[topic]")
{
	geometry_msgs::TransformStamped msg;

	g_arrayCounter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("rewriting_topic", 2);
	ros::Subscriber sub;

	int msgCounter = 0;
	geometry_msgs::TransformStampedConstPtr receivedMsg;

	sub = nh.subscribe<geometry_msgs::TransformStamped>(
		"/receive/" + nh.resolveName("rewriting_topic"), 2, [&](const geometry_msgs::TransformStampedConstPtr& msg){
			msgCounter++;
			receivedMsg = msg;
		}
	);

	msg.header.frame_id = "abc";
	msg.child_frame_id = "def";

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

	usleep(50 * 1000);

	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(msgCounter == 1)
		{
			CHECK(receivedMsg->header.frame_id == "tf_prefix/abc");
			CHECK(receivedMsg->child_frame_id == "tf_prefix/def");
			return;
		}
	}

	CAPTURE(msgCounter);
	FAIL("Not enough messages received");
}
