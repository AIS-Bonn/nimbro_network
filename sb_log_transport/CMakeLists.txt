
project(sb_log_transport)

find_package(catkin REQUIRED COMPONENTS genmsg rosgraph_msgs roscpp)

add_message_files(FILES
	LogBlock.msg
	LogMsg.msg
)

generate_messages(DEPENDENCIES
	std_msgs
	rosgraph_msgs
)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(log_sender
	src/log_sender.cpp
)
target_link_libraries(log_sender
	${catkin_LIBRARIES}
)
add_dependencies(log_sender sb_log_transport_gencpp)

add_executable(log_receiver
	src/log_receiver.cpp
)
target_link_libraries(log_receiver
	${catkin_LIBRARIES}
)
add_dependencies(log_receiver sb_log_transport_gencpp)