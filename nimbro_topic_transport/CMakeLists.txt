cmake_minimum_required(VERSION 2.8)

project(nimbro_topic_transport)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	topic_tools
	rostest
	message_generation
	config_server
)

add_message_files(FILES
	CompressedMsg.msg
	ReceiverStats.msg
	SenderStats.msg
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

find_package(plot_msgs)
if(plot_msgs_FOUND)
	include_directories(${plot_msgs_INCLUDE_DIRS})
	add_definitions(-DWITH_PLOTTING=1)
endif()

set(OPENFEC_PATH "" CACHE PATH "Path to OpenFEC (optional)")
if(OPENFEC_PATH)
	include_directories(${OPENFEC_PATH}/src/lib_common)
	set(OPENFEC_LIBRARY ${OPENFEC_PATH}/bin/Release/libopenfec.so)
	add_definitions(-DWITH_OPENFEC=1)
else()
	set(OPENFEC_LIBRARY "")
endif()

find_library(BZ2_LIBRARY bz2 REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
add_executable(udp_sender
	src/udp/topic_sender.cpp
	src/udp/udp_sender.cpp
)
add_dependencies(udp_sender
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(udp_sender
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
	${OPENFEC_LIBRARY}
)

add_executable(udp_receiver
	src/udp/udp_receiver.cpp
	src/topic_info.cpp
)
add_dependencies(udp_receiver
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(udp_receiver
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
	${OPENFEC_LIBRARY}
)

add_executable(tcp_sender
	src/tcp/tcp_sender.cpp
	src/topic_info.cpp
)
add_dependencies(tcp_sender
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(tcp_sender
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
)

add_executable(tcp_receiver
	src/tcp/tcp_receiver.cpp
	src/topic_info.cpp
)
add_dependencies(tcp_receiver
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(tcp_receiver
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
)

# Tests
add_rostest_gtest(test_comm test/topic_transport.test
	test/test_comm.cpp
)
target_link_libraries(test_comm
	${catkin_LIBRARIES}
)

add_rostest_gtest(test_comm_fec test/topic_transport_fec.test
	test/test_comm.cpp
)
target_link_libraries(test_comm_fec
	${catkin_LIBRARIES}
)
