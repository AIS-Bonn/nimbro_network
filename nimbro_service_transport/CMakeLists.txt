cmake_minimum_required(VERSION 2.8)

project(nimbro_service_transport)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	topic_tools
	message_generation
)

add_service_files(FILES
	AddTwoInts.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_executable(service_server
	src/tcp/service_server.cpp
	src/tcp/client_handler.cpp
)
target_link_libraries(service_server
	${catkin_LIBRARIES}
)

add_executable(service_client
	src/tcp/service_client.cpp
	src/common.cpp
)
target_link_libraries(service_client
	${catkin_LIBRARIES}
)

add_executable(udp_server
	src/udp/udp_server.cpp
)
target_link_libraries(udp_server
	${catkin_LIBRARIES}
)

add_executable(udp_client
	src/udp/udp_client.cpp
	src/common.cpp
)
target_link_libraries(udp_client
	${catkin_LIBRARIES}
)

# Tests
add_executable(test_server
	test/test_server.cpp
)
target_link_libraries(test_server
	${catkin_LIBRARIES}
)
add_dependencies(test_server
	${PROJECT_NAME}_generate_messages_cpp
)

add_executable(test_client
	test/test_client.cpp
)
target_link_libraries(test_client
	${catkin_LIBRARIES}
)
add_dependencies(test_client
	${PROJECT_NAME}_generate_messages_cpp
)