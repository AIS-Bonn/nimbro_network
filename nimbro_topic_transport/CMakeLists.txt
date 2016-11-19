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

# Some optional components of the NimbRo software framework
find_package(plot_msgs)
if(plot_msgs_FOUND)
	include_directories(${plot_msgs_INCLUDE_DIRS})
	add_definitions(-DWITH_PLOTTING=1)
endif()

find_package(config_server)
if(config_server_FOUND)
	include_directories(${config_server_INCLUDE_DIRS})
	add_definitions(-DWITH_CONFIG_SERVER=1)
endif()

find_package(catch_ros)

# If OpenFEC is available, we can enable FEC for the UDP sender
set(DEFAULT_OPENFEC_PATH /opt/openfec_v1.4.2)
set(OPENFEC_PATH "" CACHE PATH "Path to OpenFEC (optional)")
if(NOT OPENFEC_PATH AND IS_DIRECTORY ${DEFAULT_OPENFEC_PATH})
	set(OPENFEC_PATH ${DEFAULT_OPENFEC_PATH})
endif()
if(OPENFEC_PATH)
	include_directories(${OPENFEC_PATH}/src/lib_common)
	set(OPENFEC_LIBRARY ${OPENFEC_PATH}/bin/Release/libopenfec.so)
	add_definitions(-DWITH_OPENFEC=1)
	message(STATUS "Found and using OpenFEC at: ${OPENFEC_PATH}")
else()
	set(OPENFEC_LIBRARY "")
endif()

find_library(BZ2_LIBRARY bz2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
add_executable(udp_sender
	src/udp/topic_sender.cpp
	src/udp/udp_sender.cpp
	src/topic_info.cpp
)
add_dependencies(udp_sender
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(udp_sender
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
	${OPENFEC_LIBRARY}
	${config_server_LIBRARIES}
)

add_executable(udp_receiver
	src/udp/udp_receiver.cpp
	src/udp/topic_receiver.cpp
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
	${config_server_LIBRARIES}
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

	catch_add_rostest_node(test_comm
		test/test_comm.cpp
	)
	target_link_libraries(test_comm
		${catkin_LIBRARIES}
	)

	add_rostest(test/topic_transport.test ARGS protocol:=udp)
	add_rostest(test/topic_transport.test ARGS protocol:=tcp)

	if(OPENFEC_PATH)
		add_rostest(test/topic_transport.test ARGS protocol:=udp fec:=true)
	endif()

	catch_add_rostest_node(test_bidirectional
		test/test_bidirectional.cpp
	)
	target_link_libraries(test_bidirectional
		${catkin_LIBRARIES}
	)

	add_rostest(test/bidirectional.test ARGS allow_bidirectional:=true)
	add_rostest(test/bidirectional.test ARGS allow_bidirectional:=false)
endif()

#install
install(TARGETS udp_receiver tcp_receiver udp_sender tcp_sender
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})