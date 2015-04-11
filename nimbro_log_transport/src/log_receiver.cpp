// Receives LogBlock messages over an unreliable connection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <nimbro_log_transport/LogBlock.h>

#include <numeric>

uint32_t cur_id = (uint32_t)-1;
uint64_t cur_key = 0;
ros::Publisher pub;

void handleMsg(const nimbro_log_transport::LogBlock& block)
{
	if(block.key != cur_key)
	{
		cur_id = -1;
		cur_key = block.key;
	}

	for(size_t i = 0; i < block.msgs.size(); ++i)
	{
		const nimbro_log_transport::LogMsg& msg = block.msgs[i];

		int64_t delta = msg.id - cur_id;
		if((msg.id > cur_id && delta < std::numeric_limits<uint32_t>::max()/2)
			|| (msg.id < cur_id && ((std::numeric_limits<uint32_t>::max() - cur_id) + msg.id) < std::numeric_limits<uint32_t>::max()/2))
		{
			cur_id = msg.id;
			pub.publish(msg.msg);
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "log_receiver");

	ros::NodeHandle nh("~");
	pub = nh.advertise<rosgraph_msgs::Log>("/rosout", 1);
	ros::Subscriber sub = nh.subscribe("/rosout_transport", 1, &handleMsg);

	ros::spin();
	return 0;
}
