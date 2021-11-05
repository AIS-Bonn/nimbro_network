// Summarizes & repeats tf_static information
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_msgs/TFMessage.h>

#include <unordered_set>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_static_repeater");

	ros::NodeHandle nh{"~"};

	std::unordered_set<std::string> excludePublishers;

	{
		using Value = XmlRpc::XmlRpcValue;

		Value list;
		if(nh.getParam("exclude_publishers", list))
		{
			if(list.getType() != Value::TypeArray)
				throw std::runtime_error{"exclude_publishers should be a list"};

			for(int i = 0; i < list.size(); ++i)
			{
				XmlRpc::XmlRpcValue entry = list[i];
				if(entry.getType() != XmlRpc::XmlRpcValue::TypeString)
					throw std::runtime_error{"exclude_publishers should be a list of strings"};

				std::string s = entry;
				excludePublishers.insert(ros::names::resolve(s));
			}
		}
	}

	tf2_ros::Buffer tfBuffer;
	auto handleTransform = [&](const ros::MessageEvent<tf2_msgs::TFMessage>& event){
		// Is this publisher allowed?
		if(!excludePublishers.empty())
		{
			auto& connHeader = event.getConnectionHeader();
			auto it = connHeader.find("callerid");

			if(it == connHeader.end())
				ROS_WARN_ONCE("No callerid in connection header");
			else
			{
				if(excludePublishers.find(it->second) != excludePublishers.end())
				{
					ROS_DEBUG_NAMED("filter", "Caller ID: '%s' in exclusion list, not sending", it->second.c_str());
					return;
				}
				else
					ROS_DEBUG_NAMED("filter", "Caller ID: '%s' not in exclusion list, sending", it->second.c_str());
			}
		}

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
			if(!tfBuffer._getParent(frame, ros::Time(0), parent))
				continue;

			auto trans = tfBuffer.lookupTransform(parent, frame, ros::Time(0));
			out.transforms.push_back(trans);
		}

		pub.publish(out);
	};

	ros::Timer timer = nh.createTimer(ros::Duration(2.0), std::bind(publishTransforms));

	ros::spin();

	return 0;
}
