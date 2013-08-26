// Sends out log messages over an unreliable transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rosgraph_msgs/Log.h>
#include <boost/circular_buffer.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/timer.h>

#include <sb_log_transport/LogMsg.h>
#include <sb_log_transport/LogBlock.h>

boost::circular_buffer<sb_log_transport::LogMsg> buffer(10);
uint32_t cur_id = 0;
ros::Publisher pub;
int minLevel = rosgraph_msgs::Log::INFO;

void handleMsg(const rosgraph_msgs::Log::ConstPtr& msg)
{
	sb_log_transport::LogMsg item;
	item.id = cur_id++;
	item.msg = *msg;
	buffer.push_back(item);
}

void publish(const ros::TimerEvent&)
{
	sb_log_transport::LogBlock block;

	block.msgs.resize(buffer.size());
	std::copy(buffer.begin(), buffer.end(), block.msgs.begin());
	pub.publish(block);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "log_sender");

	ros::NodeHandle nh("~");
	nh.param<int>("min_level", minLevel, rosgraph_msgs::Log::INFO);

	pub = nh.advertise<sb_log_transport::LogBlock>("/rosout_transport", 1);
	ros::Subscriber sub = nh.subscribe("/rosout_agg", 10, &handleMsg);

	ros::Timer timer = nh.createTimer(ros::Duration(0.5), &publish);

	ros::spin();
	return 0;
}


