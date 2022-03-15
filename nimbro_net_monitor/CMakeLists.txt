
cmake_minimum_required(VERSION 3.0)
project(nimbro_net_monitor)

find_package(PkgConfig)
pkg_check_modules(MNL libmnl>=1.0.0 IMPORTED_TARGET)

if(NOT MNL_FOUND)
	message(FATAL_ERROR "nimbro_net_monitor requires libmnl")
endif()

include_directories(${MNL_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosfmt
	std_msgs
	message_generation
)

add_message_files(FILES
	ConnectionStats.msg
	InterfaceStats.msg
	NetworkStats.msg
	NodeStats.msg
	PeerStats.msg
	WifiStats.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

add_library(net_monitor_internal OBJECT
	src/route.cpp
	src/nl80211.cpp
)
target_link_libraries(net_monitor_internal PUBLIC
	PkgConfig::MNL
)


add_executable(net_monitor
	src/monitor_node.cpp
)
target_link_libraries(net_monitor
	${catkin_LIBRARIES}
	net_monitor_internal
)
add_dependencies(net_monitor
	${PROJECT_NAME}_generate_messages_cpp
)

add_executable(route_test
	src/route_test.cpp
	src/route.cpp
)
target_link_libraries(route_test
	${catkin_LIBRARIES}
	PkgConfig::MNL
)

find_package(rosfmt)

add_executable(standalone_monitor
	src/standalone_monitor.cpp
)
target_link_libraries(standalone_monitor
	net_monitor_internal
	${rosfmt_LIBRARIES}
)

add_executable(standalone_receiver
	src/standalone_receiver.cpp
)
target_link_libraries(standalone_receiver
	${catkin_LIBRARIES}
)
add_dependencies(standalone_receiver
	${PROJECT_NAME}_generate_messages_cpp
)

# GUI
add_subdirectory(src/gui)

install(TARGETS net_monitor standalone_monitor standalone_receiver
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rqt_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
