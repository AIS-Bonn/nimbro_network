cmake_minimum_required(VERSION 3.1)

project(nimbro_topic_transport)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	topic_tools
	rostest
	message_generation
)

add_message_files(FILES
	CompressedMsg.msg
	ReceiverStats.msg
	SenderStats.msg
	TopicBandwidth.msg
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

find_package(catch_ros)

# Find zstd
find_package(PkgConfig)
pkg_check_modules(ZSTD REQUIRED libzstd>=1.1.4)

include_directories(${ZSTD_INCLUDE_DIRS})
include_directories(contrib)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")

add_library(nimbro_topic_transport
	contrib/wirehair/wirehair.cpp
	contrib/wirehair/gf256.cpp
	contrib/wirehair/WirehairCodec.cpp
	contrib/wirehair/WirehairTools.cpp
	src/sender/compressor.cpp
	src/sender/get_route.cpp
	src/sender/packetizer.cpp
	src/sender/subscriber.cpp
	src/sender/sender.cpp
	src/sender/tcp_sender.cpp
	src/sender/udp_sender.cpp
	src/receiver/decompressor.cpp
	src/receiver/depacketizer.cpp
	src/receiver/publisher.cpp
	src/receiver/receiver.cpp
	src/receiver/tcp_receiver.cpp
	src/receiver/udp_receiver.cpp
	src/topic_info.cpp
	src/thread_pool.cpp
)
target_link_libraries(nimbro_topic_transport
	${catkin_LIBRARIES}
	${ZSTD_LIBRARIES}
)

add_executable(sender
	src/sender/main.cpp
)
target_link_libraries(sender
	${catkin_LIBRARIES}
	nimbro_topic_transport
)

add_executable(receiver
	src/receiver/main.cpp
)
target_link_libraries(receiver
	${catkin_LIBRARIES}
	nimbro_topic_transport
)

add_executable(transceiver
	src/transceiver.cpp
)
target_link_libraries(transceiver
	${catkin_LIBRARIES}
	nimbro_topic_transport
)

# Tools
add_executable(action_proxy
	src/action_proxy.cpp
	src/topic_info.cpp
)
target_link_libraries(action_proxy
	${catkin_LIBRARIES}
)

# GUI
add_subdirectory(src/gui)

# Tests
if(catch_ros_FOUND)
	include_directories(${catch_ros_INCLUDE_DIRS})

	catch_add_test(unit_tests
		test/unit/test_packetizer.cpp
		test/unit/test_compression.cpp
	)
	target_link_libraries(unit_tests
		nimbro_topic_transport
	)

	catch_add_rostest_node(test_comm
		test/test_comm.cpp
	)
	target_link_libraries(test_comm
		${catkin_LIBRARIES}
	)

	add_rostest(test/topic_transport.test ARGS port:=5778 fec:=false)
	add_rostest(test/topic_transport.test ARGS port:=5779 fec:=true)
	add_rostest(test/transceiver.test ARGS port:=5780 fec:=false)
endif()

#install
install(TARGETS nimbro_topic_transport sender receiver action_proxy transceiver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES utils/get_msg_def.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
