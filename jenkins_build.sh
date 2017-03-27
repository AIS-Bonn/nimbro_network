#!/bin/bash -e

# Build!
. /opt/ros/indigo/setup.bash
stdbuf -oL catkin build --no-status --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall"

# Run tests!
export ROS_TEST_RESULTS_DIR=$(pwd)/build/test_results
stdbuf -oL catkin build --no-status --catkin-make-args run_tests

