// Unit tests for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <catch_ros/catch.hpp>

#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/init.h>

#include <std_msgs/Int64.h>
#include <std_msgs/UInt64MultiArray.h>

#include <geometry_msgs/TransformStamped.h>

TEST_CASE("simple", "[topic]")
{
	std_msgs::Int64 msg;

	int msgCounter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub;
	ros::Subscriber sub;

	auto handler = [&](const std_msgs::Int64ConstPtr& msg) {
		ROS_INFO("simple with ID %ld", msg->data);
		REQUIRE(msgCounter == msg->data);
		msgCounter++;
	};

	SECTION("simple")
	{
		pub = nh.advertise<std_msgs::Int64>("test_topic", 2);
		sub = nh.subscribe<std_msgs::Int64>("/receive/" + nh.resolveName("test_topic"), 20,
			handler
		);
	}

	SECTION("remove prefix")
	{
		pub = nh.advertise<std_msgs::Int64>("/odd_prefix/" + nh.resolveName("unprefix_me"), 2);
		sub = nh.subscribe<std_msgs::Int64>("/receive/" + nh.resolveName("unprefix_me"), 20,
			handler
		);
	}

	int timeout = 300;
	while(pub.getNumSubscribers() == 0 || sub.getNumPublishers() == 0)
	{
		ros::spinOnce();
		if(--timeout == 0)
		{
			CAPTURE(pub.getNumSubscribers());
			CAPTURE(sub.getNumPublishers());
			FAIL("sender did not subscribe or receiver did not advertise on our topic");
			return;
		}

		usleep(20 * 1000);
	}

	usleep(50 * 1000);

	msg.data = 0;
	pub.publish(msg);

	for(int i = 0; i < 50; ++i)
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

		if(msgCounter == 2)
			return;
	}

	CAPTURE(msgCounter);
	FAIL("Not enough messages received on topic " << sub.getTopic());
}

TEST_CASE("array", "[topic]")
{
	std_msgs::UInt64MultiArray msg;

	int msgCounter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::UInt64MultiArray>("array_topic", 2);
	ros::Subscriber sub;

	SECTION("small")
	{
		for(int i = 0; i < 512; ++i)
			msg.data.push_back(i);

		sub = nh.subscribe<std_msgs::UInt64MultiArray>("/receive/" + nh.resolveName("array_topic"), 2,
			[&](const std_msgs::UInt64MultiArrayConstPtr& msg){
				ROS_INFO("array");
				REQUIRE(msg->data.size() == 512);
				for(int i = 0; i < 512; ++i)
				{
					REQUIRE(msg->data[i] == i);
				}
				msgCounter++;
			}
		);
	}
	SECTION("huge")
	{
		constexpr int HUGE_SIZE = 3 * 1024;

		for(int i = 0; i < HUGE_SIZE; ++i)
			msg.data.push_back(i);

		sub = nh.subscribe<std_msgs::UInt64MultiArray>("/receive/" + nh.resolveName("array_topic"), 2,
			[&](const std_msgs::UInt64MultiArrayConstPtr& msg) {
				ROS_INFO("huge array");
				REQUIRE(msg->data.size() == HUGE_SIZE);
				for(int i = 0; i < HUGE_SIZE; ++i)
				{
					REQUIRE(msg->data[i] == i);
				}
				msgCounter++;
			}
		);
	}

	int timeout = 300;
	while(pub.getNumSubscribers() == 0 || sub.getNumPublishers() == 0)
	{
		ros::spinOnce();
		if(--timeout == 0)
		{
			CAPTURE(pub.getNumSubscribers());
			CAPTURE(sub.getNumPublishers());
			FAIL("sender did not subscribe or receiver did not advertise on our topic");
			return;
		}

		usleep(20 * 1000);
	}

	usleep(10 * 1000);

	msgCounter = 0;
	pub.publish(msg);

	for(int i = 0; i < 50; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	INFO("Received after first message was sent: " << msgCounter);
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(msgCounter == 2)
			return;
	}

	CAPTURE(msgCounter);
	FAIL("Not enough messages received");
}

TEST_CASE("rewriting", "[topic]")
{
	geometry_msgs::TransformStamped msg;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("rewriting_topic", 2);
	ros::Subscriber sub;

	int msgCounter = 0;
	geometry_msgs::TransformStampedConstPtr receivedMsg;

	sub = nh.subscribe<geometry_msgs::TransformStamped>(
		"/receive/" + nh.resolveName("rewriting_topic"), 2, [&](const geometry_msgs::TransformStampedConstPtr& msg){
			ROS_INFO("rewrite");
			msgCounter++;
			receivedMsg = msg;
		}
	);

	msg.header.frame_id = "abc";
	msg.child_frame_id = "def";

	ros::WallTime t0 = ros::WallTime::now();
	int timeout = 300;
	while(pub.getNumSubscribers() == 0 || sub.getNumPublishers() == 0)
	{
		ros::spinOnce();
		ROS_INFO("pub: %u, sub: %u", pub.getNumSubscribers(), sub.getNumPublishers());
		if(--timeout == 0)
		{
			CAPTURE(pub.getNumSubscribers());
			CAPTURE(sub.getNumPublishers());
			FAIL("sender did not subscribe or receiver did not advertise on our topic");
			return;
		}

		usleep(20 * 1000);
	}
	ros::WallTime t1 = ros::WallTime::now();
	ROS_INFO("Waiting for connection took %f s", (t1 - t0).toSec());

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
