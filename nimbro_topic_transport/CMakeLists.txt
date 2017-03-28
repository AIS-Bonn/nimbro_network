cmake_minimum_required(VERSION 2.8)

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
	CMAKE_ARGS "-DCMAKE_C_FLAGS_RELEASE=-O3 -DNDEBUG -w" # openfec produces a lot of warnings :-(
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

### BEGIN ZSTD
ExternalProject_Add(
	zstd
	URL "https://github.com/facebook/zstd/archive/v1.1.4.tar.gz"
	CONFIGURE_COMMAND ""
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND ""
)

ExternalProject_Get_Property(zstd SOURCE_DIR)
set(ZSTD_LIBRARY ${SOURCE_DIR}/lib/libzstd.so)
include_directories(${SOURCE_DIR}/lib)

install(FILES
	${SOURCE_DIR}/lib/libzstd.so.1.1.4
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set(ZSTD_LINK_DEST ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/libzstd.so.1)
install(CODE "execute_process(COMMAND ln -s libzstd.so.1.1.4 ${ZSTD_LINK_DEST})")
add_definitions(-DWITH_ZSTD=1)
### END ZSTD

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_library(nimbro_topic_transport
	src/sender/compressor.cpp
	src/sender/packetizer.cpp
	src/sender/subscriber.cpp
	src/sender/udp_sender.cpp
	src/sender/tcp_sender.cpp
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
	${ZSTD_LIBRARY}
)
add_dependencies(nimbro_topic_transport openfec zstd)


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
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_SRCS
	src/gui/topic_gui.h
	src/gui/dot_widget.h
)

qt4_wrap_cpp(BAND_MOC_SRCS
	src/gui/bandwidth_gui.h
	src/gui/contrib/qcustomplot/qcustomplot.h
)

add_library(topic_gui
	${MOC_SRCS}
	src/gui/topic_gui.cpp
	src/gui/dot_widget.cpp
)

add_library(bandwidth_gui
	${BAND_MOC_SRCS}
	src/gui/bandwidth_gui.cpp
	src/gui/contrib/qcustomplot/qcustomplot.cpp
)


add_dependencies(topic_gui
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(topic_gui
	${QT_LIBRARIES}
)

add_dependencies(bandwidth_gui
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(bandwidth_gui
	${QT_LIBRARIES}
	yaml-cpp
)

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
