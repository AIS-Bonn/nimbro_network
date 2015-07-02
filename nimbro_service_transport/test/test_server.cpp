// Small node providing test services
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <nimbro_service_transport/AddTwoInts.h>

bool handleEmptySuccess(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	return true;
}

bool handleEmptyFailure(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	return false;
}

bool handleAddTwoInts(nimbro_service_transport::AddTwoIntsRequest& req,
	nimbro_service_transport::AddTwoIntsResponse& response)
{
	response.sum = req.a + req.b;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_server");

	ros::NodeHandle nh("~");

	ros::ServiceServer srv_emptySuccess = nh.advertiseService("empty_success",
		&handleEmptySuccess
	);
	ros::ServiceServer srv_emptyFailure = nh.advertiseService("empty_failure",
		&handleEmptyFailure
	);

	ros::ServiceServer srv_add = nh.advertiseService("add",
		&handleAddTwoInts
	);

	ros::spin();

	return 0;
}
