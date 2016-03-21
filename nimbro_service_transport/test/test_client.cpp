// Unit test client
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <nimbro_service_transport/AddTwoInts.h>

#include <catch_ros/catch.hpp>

TEST_CASE("empty success", "[empty]")
{
	std_srvs::Empty srv;
	ROS_INFO("Calling empty_success");
	REQUIRE(ros::service::waitForService("/remote/test_server/empty_success", 2000));
	REQUIRE(ros::service::call("/remote/test_server/empty_success", srv));
}

TEST_CASE("empty failure", "[empty]")
{
	std_srvs::Empty srv;
	ROS_INFO("Calling empty_failure");
	REQUIRE(ros::service::waitForService("/remote/test_server/empty_failure", 2000));
	REQUIRE(!ros::service::call("/remote/test_server/empty_failure", srv));
}

TEST_CASE("add two ints", "[add]")
{
	nimbro_service_transport::AddTwoInts srv;
	srv.request.a = 7;
	srv.request.b = 5;
	ROS_INFO("Calling add");
	REQUIRE(ros::service::waitForService("/remote/test_server/add", 2000));
	REQUIRE(ros::service::call("/remote/test_server/add", srv));
	REQUIRE(srv.response.sum == 12);
}
