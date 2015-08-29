// Sends out log messages over an unreliable transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rosgraph_msgs/Log.h>
#include <boost/circular_buffer.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/timer.h>

#include <random>

#include <nimbro_log_transport/LogMsg.h>
#include <nimbro_log_transport/LogBlock.h>

boost::circular_buffer<nimbro_log_transport::LogMsg> buffer;
uint32_t cur_id = 0;
uint64_t key = 0;
ros::Publisher pub;
int minLevel = rosgraph_msgs::Log::INFO;

void handleMsg(const rosgraph_msgs::Log::ConstPtr& msg)
{
	nimbro_log_transport::LogMsg item;
	item.id = cur_id++;
	item.msg = *msg;
	buffer.push_back(item);
}

void publish(const ros::TimerEvent&)
{
	nimbro_log_transport::LogBlock block;

	block.key = key;
	block.msgs.resize(buffer.size());
	std::copy(buffer.begin(), buffer.end(), block.msgs.begin());
	pub.publish(block);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "log_sender");

	ros::NodeHandle nh("~");
	nh.param<int>("min_level", minLevel, rosgraph_msgs::Log::INFO);

	int bufferSize;
	nh.param<int>("buffer_size", bufferSize, 10);

	double rate;
	nh.param<double>("rate", rate, 2.0);

	buffer.resize(bufferSize);

	std::random_device rd;

    std::mt19937_64 e2(rd());

    std::uniform_int_distribution<uint64_t> dist;

	key = dist(e2);

	pub = nh.advertise<nimbro_log_transport::LogBlock>("/rosout_transport", 1);
	ros::Subscriber sub = nh.subscribe("/rosout_agg", bufferSize, &handleMsg);

	ros::Timer timer = nh.createTimer(ros::Duration(1.0 / rate), &publish);

	ros::spin();
	return 0;
}
