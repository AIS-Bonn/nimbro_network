cmake_minimum_required(VERSION 2.8)

project(tf_throttle)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_throttle
	src/tf_throttle.cpp
)

target_link_libraries(tf_throttle
	${catkin_LIBRARIES}
)
