// Unit tests for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <gtest/gtest.h>

#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/init.h>

#include <std_msgs/Int64.h>
#include <std_msgs/UInt64MultiArray.h>

int g_counter = 0;

void handle_simple(const std_msgs::Int64& msg)
{
	ROS_ERROR("handler");
	ASSERT_EQ(g_counter, msg.data);
	g_counter++;
}

int g_arrayCounter = 0;

void handle_array(const std_msgs::UInt64MultiArray& msg)
{
	ROS_ERROR("array");

	ASSERT_EQ(512, msg.data.size());
	for(int i = 0; i < 512; ++i)
	{
		ASSERT_EQ(i, msg.data[i]);
	}
	g_arrayCounter++;
}

TEST(TopicTransportTest, test_simple)
{
	std_msgs::Int64 msg;

	g_counter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::Int64>("/test_topic", 2);
	ros::Subscriber sub = nh.subscribe("/receive/test_topic", 2, &handle_simple);

	sleep(1);

	msg.data = 0;
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	usleep(1000);
	sleep(1);

	int timeout = 1000;
	while(pub.getNumSubscribers() == 0)
	{
		ros::spinOnce();
		if(--timeout == 0)
			FAIL();

		usleep(200);
	}

	msg.data = 0;
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	msg.data = 1;
	pub.publish(msg);

	ROS_ERROR("published");
	sleep(2);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(g_counter == 2)
			return;
	}

	FAIL();
}

TEST(TopicTransportTest, test_array)
{
	std_msgs::UInt64MultiArray msg;

	g_counter = 0;

	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::UInt64MultiArray>("/array_topic", 2);
	ros::Subscriber sub = nh.subscribe("/receive/array_topic", 2, &handle_array);

	sleep(1);

	for(int i = 0; i < 512; ++i)
		msg.data.push_back(i);
	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	usleep(1000);
	sleep(1);

	int timeout = 1000;
	while(pub.getNumSubscribers() == 0)
	{
		ros::spinOnce();
		if(--timeout == 0)
			FAIL();

		usleep(200);
	}

	pub.publish(msg);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);
	}

	pub.publish(msg);

	ROS_ERROR("array published");
	sleep(2);

	for(int i = 0; i < 1000; ++i)
	{
		ros::spinOnce();
		usleep(1000);

		if(g_arrayCounter == 2)
			return;
	}

	FAIL();
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_comm");

	return RUN_ALL_TESTS();
}
