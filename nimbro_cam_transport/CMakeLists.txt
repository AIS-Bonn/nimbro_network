
cmake_minimum_required(VERSION 3.10)
project(nimbro_cam_transport)

find_package(catkin REQUIRED COMPONENTS
	image_transport
	roscpp
	cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

find_library(x264_LIBRARY x264)

if(NOT x264_LIBRARY)
	message(WARNING "x264 library not found, not building senders...")
endif()

find_library(LIBAVUTIL_LIBRARY avutil HINTS /opt/ffmpeg/lib)
find_library(LIBAVCODEC_LIBRARY avcodec HINTS /opt/ffmpeg/lib)
find_library(LIBSWSCALE_LIBRARY swscale HINTS /opt/ffmpeg/lib)

find_path(FFMPEG_INCLUDE_PATH "libavcodec/avcodec.h" HINTS "/opt/ffmpeg/include")
include_directories(${FFMPEG_INCLUDE_PATH})

message(STATUS "libavcodec: ${LIBAVCODEC_LIBRARY} and ${FFMPEG_INCLUDE_PATH}")
if(NOT LIBAVCODEC_LIBRARY OR NOT FFMPEG_INCLUDE_PATH)
	message(WARNING "Could not find ffmpeg libraries, not building receivers...")
endif()

if(x264_LIBRARY)
	add_executable(cam_sender
		src/sender.cpp
		src/rgb_to_yuv420.cpp
	)
	target_link_libraries(cam_sender
		${catkin_LIBRARIES}
		${x264_LIBRARY}
	)

	install(TARGETS cam_sender
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()

if(LIBAVCODEC_LIBRARY AND EXISTS ${FFMPEG_INCLUDE_PATH})
	add_executable(cam_receiver
		src/receiver.cpp
	)
	target_link_libraries(cam_receiver
		${catkin_LIBRARIES}
		${LIBAVUTIL_LIBRARY}
		${LIBAVCODEC_LIBRARY}
		${LIBSWSCALE_LIBRARY}
	)

	install(TARGETS cam_receiver
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()

# Register unit tests
if(CATKIN_ENABLE_TESTING AND TARGET cam_sender AND TARGET cam_receiver)
	# Integration tests
	find_package(rostest REQUIRED)
	add_rostest(test/basic.test DEPENDENCIES cam_sender cam_receiver)
endif()
