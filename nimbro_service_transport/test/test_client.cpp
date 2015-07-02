// Unit test client
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <nimbro_service_transport/AddTwoInts.h>

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

TEST_CASE("empty success", "[empty]")
{
	std_srvs::Empty srv;
	ROS_INFO("Calling empty_success");
	REQUIRE(ros::service::call("/remote/test_server/empty_success", srv));
}

TEST_CASE("empty failure", "[empty]")
{
	std_srvs::Empty srv;
	ROS_INFO("Calling empty_failure");
	REQUIRE(!ros::service::call("/remote/test_server/empty_failure", srv));
}

TEST_CASE("add two ints", "[add]")
{
	nimbro_service_transport::AddTwoInts srv;
	srv.request.a = 7;
	srv.request.b = 5;
	ROS_INFO("Calling add");
	REQUIRE(ros::service::call("/remote/test_server/add", srv));
	REQUIRE(srv.response.sum == 12);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_client");

	Catch::Session session;

	std::string test_output;
	for(int i = 1; i < argc; ++i)
	{
		if(strncmp(argv[i], "--gtest_output=xml:", 19) == 0)
		{
			test_output = argv[i] + 19;
		}
	}

	if(!test_output.empty())
	{
		session.configData().reporterName = "junit";
		session.configData().outputFilename = test_output;
	}

	return session.run();
}
