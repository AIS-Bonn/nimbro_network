// Receive msg from tf and measure latency
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>

#include <boost/program_options.hpp>

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
	namespace po = boost::program_options;

	ros::init(argc, argv, "measure_tf_latency", ros::init_options::AnonymousName);

	po::options_description desc("Options");
	desc.add_options()
		("help", "This help message")
		("udp", "Use UDP transport")
		("nodelay", "Disable Nagle's algorithm")
	;

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
          options(desc).run(), vm);

	if(vm.count("help"))
	{
		std::cerr << "Usage: measure_tf_latency\n";
		std::cerr << desc << "\n";
		return 1;
	}

	po::notify(vm);

	ros::TransportHints hints;

	if(vm.count("udp"))
		hints = hints.udp();

	if(vm.count("nodelay"))
		hints = hints.tcpNoDelay();

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/tf", 1, &handleMsg, hints);

	ros::spin();

	return 0;
}

