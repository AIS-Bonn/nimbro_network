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

# If OpenFEC is available, we can enable FEC for the UDP sender
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
	${config_server_LIBRARIES}
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

# GUI
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_SRCS
	src/gui/topic_gui.h
	src/gui/dot_widget.h
)

qt4_wrap_cpp(BAND_MOC_SRCS
	src/gui/bandwidth_gui.h
	src/gui/contrib/qcustomplot.h
)

add_library(topic_gui
	${MOC_SRCS}
	src/gui/topic_gui.cpp
	src/gui/dot_widget.cpp
)

add_library(bandwidth_gui
	${BAND_MOC_SRCS}
	src/gui/bandwidth_gui.cpp
	src/gui/contrib/qcustomplot.cpp
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
