
project(sb_udp)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	topic_tools
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

execute_process(
	COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
	OUTPUT_VARIABLE GCC_VERSION
)

if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
	set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
	add_executable(udp_sender
		src/topic_sender.cpp
		src/udp_sender.cpp
	)

	target_link_libraries(udp_sender
		${catkin_LIBRARIES}
	)

	add_executable(udp_receiver
		src/udp_receiver.cpp
	)

	target_link_libraries(udp_receiver
		${catkin_LIBRARIES}
	)
else()
	message(WARNING "C++11 not supported, compiler version ${GCC_VERSION}")
endif()
