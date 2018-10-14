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

include(ExternalProject)

### BEGIN OPENFEC
ExternalProject_Add(
	openfec
	URL http://openfec.org/files/openfec_v1_4_2.tgz
	URL_HASH SHA256=ccf1252b548505d198de768609c08c722a35e580c53b843e4372b7c52674ce12
	CMAKE_ARGS "-DCMAKE_C_FLAGS_RELEASE=-O3 -DNDEBUG -w" # openfec produces a lot of warnings :-(
	BUILD_COMMAND $(MAKE)
	INSTALL_COMMAND "" # openfec does not provide an install target :-(
)

ExternalProject_Get_Property(openfec SOURCE_DIR)
set(OPENFEC_PATH "${SOURCE_DIR}")

add_definitions(-DWITH_OPENFEC=1)
include_directories(${OPENFEC_PATH}/src/lib_common)
set(OPENFEC_LIB_PATH ${OPENFEC_PATH}/bin/Release)
set(OPENFEC_LIBRARY ${OPENFEC_LIB_PATH}/libopenfec.so)
install(FILES
	${OPENFEC_PATH}/bin/Release/libopenfec.so.1.4.2
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(OPENFEC_LINK_DEST ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/libopenfec.so.1)
install(CODE "execute_process(COMMAND ln -s libopenfec.so.1.4.2 ${OPENFEC_LINK_DEST})")

if(NOT ";${CMAKE_INSTALL_RPATH};" MATCHES ";${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION};")
	list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")
endif()
message(STATUS "Found and using OpenFEC at: ${OPENFEC_PATH}")

### END OPENFEC

# Find zstd
find_package(PkgConfig)
pkg_check_modules(ZSTD REQUIRED libzstd>=1.1.4)

include_directories(${ZSTD_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_library(nimbro_topic_transport
	src/sender/compressor.cpp
	src/sender/packetizer.cpp
	src/sender/subscriber.cpp
	src/sender/udp_sender.cpp
	src/sender/tcp_sender.cpp
	src/sender/get_route.cpp
	src/receiver/decompressor.cpp
	src/receiver/depacketizer.cpp
	src/receiver/publisher.cpp
	src/receiver/udp_receiver.cpp
	src/receiver/tcp_receiver.cpp
	src/topic_info.cpp
	src/thread_pool.cpp
)
target_link_libraries(nimbro_topic_transport
	${catkin_LIBRARIES}
	${OPENFEC_LIBRARY}
	${ZSTD_LIBRARIES}
)
add_dependencies(nimbro_topic_transport openfec)


add_executable(sender
	src/sender/sender.cpp
)
target_link_libraries(sender
	${catkin_LIBRARIES}
	nimbro_topic_transport
)

add_executable(receiver
	src/receiver/receiver.cpp
)
target_link_libraries(receiver
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
endif()

#install
install(TARGETS nimbro_topic_transport sender receiver action_proxy
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
