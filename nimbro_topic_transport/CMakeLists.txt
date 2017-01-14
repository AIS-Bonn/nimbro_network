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

include(ExternalProject)
ExternalProject_Add(
	openfec
	URL http://openfec.org/files/openfec_v1_4_2.tgz
	BUILD_COMMAND make
	INSTALL_COMMAND "" # openfec does not provide an install target :-(
)

ExternalProject_Get_Property(openfec SOURCE_DIR)
set(OPENFEC_PATH "${SOURCE_DIR}")

add_definitions(-DWITH_OPENFEC=1)
include_directories(${OPENFEC_PATH}/src/lib_common)
set(OPENFEC_LIB_PATH ${OPENFEC_PATH}/bin/Release)
set(OPENFEC_LIBRARY ${OPENFEC_LIB_PATH}/libopenfec.so)
install(FILES
	${OPENFEC_PATH}/bin/Release/libopenfec.so.1
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
if(NOT ";${CMAKE_INSTALL_RPATH};" MATCHES ";${OPENFEC_LIB_PATH};")
	list(APPEND CMAKE_INSTALL_RPATH ${OPENFEC_LIB_PATH})
endif()
message(STATUS "Found and using OpenFEC at: ${OPENFEC_PATH}")

find_library(BZ2_LIBRARY bz2 REQUIRED)

# Check if zstd is available
find_library(ZSTD_LIBRARY zstd)
if(ZSTD_LIBRARY)
	add_definitions(-DWITH_ZSTD=1)
	set(ZSTD_LIBRARIES ${ZSTD_LIBRARY})
else()
	message(WARNING "Consider installing zstd for fast compression")
	set(ZSTD_LIBRARIES "")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_library(nimbro_topic_transport
	src/sender/compressor.cpp
	src/sender/packetizer.cpp
	src/receiver/decompressor.cpp
	src/receiver/depacketizer.cpp
	src/topic_info.cpp
)
target_link_libraries(nimbro_topic_transport
	${catkin_LIBRARIES}
	${OPENFEC_LIBRARY}
	${ZSTD_LIBRARIES}
	${BZ2_LIBRARY}
)
add_dependencies(nimbro_topic_transport openfec)


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
endif()

#install
# install(TARGETS udp_receiver tcp_receiver udp_sender tcp_sender
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
