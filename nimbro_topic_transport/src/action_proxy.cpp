// Proxy node for action topics
// actionlib expects that all action topics are handled by a single node
// -- so republish / resubscribe them from here.
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <topic_tools/shape_shifter.h>

#include "topic_info.h"

ros::Subscriber m_sub_goal;
ros::Publisher m_pub_goal;

void handleGoal(const topic_tools::ShapeShifter::ConstPtr& goal)
{
	m_pub_goal.publish(goal);
}

ros::Subscriber m_sub_cancel;
ros::Publisher m_pub_cancel;

void handleCancel(const actionlib_msgs::GoalID& id)
{
	m_pub_cancel.publish(id);
}

ros::Subscriber m_sub_result;
ros::Publisher m_pub_result;

void handleResult(const topic_tools::ShapeShifter::ConstPtr& result)
{
	m_pub_result.publish(result);
}

ros::Subscriber m_sub_status;
ros::Publisher m_pub_status;

void handleStatus(const actionlib_msgs::GoalStatusArray& status)
{
	m_pub_status.publish(status);
}

ros::Subscriber m_sub_feedback;
ros::Publisher m_pub_feedback;

void handleFeedback(const topic_tools::ShapeShifter::ConstPtr& fb)
{
	m_pub_feedback.publish(fb);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_proxy");

	ros::NodeHandle nh("~");

	std::string type;
	if(!nh.getParam("type", type))
	{
		ROS_FATAL("need type parameter");
		return 1;
	}

	std::string input;
	if(!nh.getParam("input", input))
	{
		ROS_FATAL("need input parameter");
		return 1;
	}

	std::string output;
	if(!nh.getParam("output", output))
	{
		ROS_FATAL("need output parameter");
		return 1;
	}

	{
		ros::AdvertiseOptions ops;
		ops.topic = input + "/goal";
		ops.datatype = type + "ActionGoal";
		ops.md5sum = nimbro_topic_transport::topic_info::getMd5Sum(ops.datatype);

		m_pub_goal = nh.advertise(ops);

		ros::SubscribeOptions subops;
		subops.init<topic_tools::ShapeShifter>(output + "/goal", 10, boost::bind(&handleGoal, _1));
		subops.datatype = type + "ActionGoal";
		subops.md5sum = nimbro_topic_transport::topic_info::getMd5Sum(subops.datatype);
		m_sub_goal = nh.subscribe(subops);
	}

	{
		m_pub_cancel = nh.advertise<actionlib_msgs::GoalID>(input + "/cancel", 1);
		m_sub_cancel = nh.subscribe(output + "/cancel", 10, &handleCancel);
	}

	{
		ros::AdvertiseOptions ops;
		ops.topic = output + "/result";
		ops.datatype = type + "ActionResult";
		ops.md5sum = nimbro_topic_transport::topic_info::getMd5Sum(ops.datatype);

		m_pub_result = nh.advertise(ops);

		ros::SubscribeOptions subops;
		subops.init<topic_tools::ShapeShifter>(input + "/result", 10, boost::bind(&handleResult, _1));
		subops.datatype = type + "ActionResult";
		subops.md5sum = nimbro_topic_transport::topic_info::getMd5Sum(subops.datatype);
		m_sub_result = nh.subscribe(subops);
	}

	{
		ros::AdvertiseOptions ops;
		ops.topic = output + "/feedback";
		ops.datatype = type + "ActionFeedback";
		ops.md5sum = nimbro_topic_transport::topic_info::getMd5Sum(ops.datatype);

		m_pub_feedback = nh.advertise(ops);

		ros::SubscribeOptions subops;
		subops.init<topic_tools::ShapeShifter>(input + "/feedback", 10, boost::bind(&handleFeedback, _1));
		subops.datatype = type + "ActionFeedback";
		subops.md5sum = nimbro_topic_transport::topic_info::getMd5Sum(subops.datatype);
		m_sub_feedback = nh.subscribe(subops);
	}

	{
		m_pub_status = nh.advertise<actionlib_msgs::GoalStatusArray>(output + "/status", 10);
		m_sub_status = nh.subscribe(input + "/status", 10, &handleStatus);
	}

	ros::spin();

	return 0;
}
