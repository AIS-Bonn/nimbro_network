cmake_minimum_required(VERSION 2.8)

project(nimbro_tf_throttle)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_throttle
	src/tf_throttle.cpp
)

target_link_libraries(tf_throttle
	${catkin_LIBRARIES}
)

install(TARGETS tf_throttle
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
