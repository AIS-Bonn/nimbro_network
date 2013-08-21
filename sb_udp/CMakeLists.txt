
project(sb_udp)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	topic_tools
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

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
