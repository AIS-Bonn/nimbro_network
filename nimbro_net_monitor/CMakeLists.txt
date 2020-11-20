
cmake_minimum_required(VERSION 3.0)
project(nimbro_net_monitor)

find_package(PkgConfig)
pkg_check_modules(MNL libmnl>=1.0.0)

if(MNL_FOUND)
	include_directories(${MNL_INCLUDE_DIRS})
	add_definitions(-DWITH_LIBMNL=1)
else()
	message(WARNING "Install libmnl-dev for network statistics by interface")
	set(MNL_LIBRARIES "")
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
)

add_message_files(FILES
	ConnectionStats.msg
	InterfaceStats.msg
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
	src/route.cpp
)
target_link_libraries(net_monitor
	${catkin_LIBRARIES}
	${MNL_LIBRARIES}
)
add_dependencies(net_monitor
	${PROJECT_NAME}_generate_messages_cpp
)

if(MNL_FOUND)
	include_directories(${MNL_INCLUDE_DIRS})

	add_executable(route_test
		src/route_test.cpp
		src/route.cpp
	)
	target_link_libraries(route_test
		${catkin_LIBRARIES}
		${MNL_LIBRARIES}
	)
endif()

# GUI
add_subdirectory(src/gui)

install(TARGETS net_monitor route_test
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rqt_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
