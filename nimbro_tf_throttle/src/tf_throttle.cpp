// Throttle /tf bandwidth
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf/transform_listener.h>

#include <tf2_msgs/TFMessage.h>

#include <boost/foreach.hpp>

bool g_applyPrefix = false;
std::string g_framePrefix;

boost::scoped_ptr<tf::TransformListener> g_tf;
ros::Publisher pub;

void sendTransforms()
{
	std::vector<std::string> frames;
	g_tf->getFrameStrings(frames);

	tf2_msgs::TFMessage msg;
	msg.transforms.reserve(frames.size());

	BOOST_FOREACH(std::string frame, frames)
	{
		std::string parentFrame;
		if(!g_tf->getParent(frame, ros::Time(0), parentFrame))
			continue;

		tf::StampedTransform transform;
		try
		{
			g_tf->lookupTransform(
				parentFrame,
				frame,
				ros::Time(0),
				transform
			);
		}
		catch(tf::TransformException&)
		{
			continue;
		}

		geometry_msgs::TransformStamped m;
		tf::transformStampedTFToMsg(transform, m);

                if ( g_applyPrefix )
                {
                    m.header.frame_id = g_framePrefix+"_"+m.header.frame_id;
                    m.child_frame_id = g_framePrefix+"_"+m.child_frame_id;
                }

		msg.transforms.push_back(m);
	}

	pub.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_throttle");

	ros::NodeHandle nh("~");

	g_tf.reset(new tf::TransformListener(nh, ros::Duration(10.0)));

	double rate;
	nh.param("rate", rate, 4.0);
        nh.param<std::string>("prefix", g_framePrefix, "");
        g_applyPrefix = g_framePrefix != "";

	ros::Timer timer = nh.createTimer(
		ros::Duration(1.0 / rate),
		boost::bind(&sendTransforms)
	);
	pub = nh.advertise<tf2_msgs::TFMessage>("tf", 1);

	ros::spin();

	g_tf.reset();

	return 0;
}
