
project(nimbro_service_transport)

find_package(catkin REQUIRED COMPONENTS roscpp roslib topic_tools)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(service_server
	src/service_server.cpp
	src/client_handler.cpp
)
target_link_libraries(service_server
	${catkin_LIBRARIES}
)

add_executable(service_client
	src/service_client.cpp
)
target_link_libraries(service_client
	${catkin_LIBRARIES}
)
