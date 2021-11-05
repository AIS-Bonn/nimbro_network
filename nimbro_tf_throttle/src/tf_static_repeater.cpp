// Summarizes & repeats tf_static information
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_msgs/TFMessage.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_static_repeater");

	ros::NodeHandle nh{"~"};

	tf2_ros::Buffer tfBuffer;
	auto handleTransform = [&](const ros::MessageEvent<tf2_msgs::TFMessage>& event){
		auto msg = event.getMessage();

		for(auto& trans : msg->transforms)
			tfBuffer.setTransform(trans, "", true);
	};

	ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("/tf_static", 100, boost::function<void(const ros::MessageEvent<tf2_msgs::TFMessage>&)>{handleTransform});

	ros::Publisher pub = nh.advertise<tf2_msgs::TFMessage>("aggregated", 1, true);


	auto publishTransforms = [&](){
		std::vector<std::string> ids;
		tfBuffer._getFrameStrings(ids);

		tf2_msgs::TFMessage out;

		for(auto& frame : ids)
		{
			std::string parent;
			tfBuffer._getParent(frame, ros::Time(0), parent);

			auto trans = tfBuffer.lookupTransform(parent, frame, ros::Time(0));
			out.transforms.push_back(trans);
		}

		pub.publish(out);
	};

	ros::Timer timer = nh.createTimer(ros::Duration(2.0), std::bind(publishTransforms));

	ros::spin();

	return 0;
}
