
cmake_minimum_required(VERSION 3.0)
project(nimbro_net_monitor)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
)

add_message_files(FILES
	ConnectionStats.msg
	NetworkStats.msg
	NodeStats.msg
	PeerStats.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 11)
add_executable(net_monitor
	src/monitor_node.cpp
)
target_link_libraries(net_monitor
	${catkin_LIBRARIES}
)
add_dependencies(net_monitor
	${PROJECT_NAME}_generate_messages_cpp
)
