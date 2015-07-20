
cmake_minimum_required(VERSION 2.8)
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

set(FFMPEG_INCLUDE_PATH "/opt/ffmpeg/include" CACHE PATH "path to libavcodec/avcodec.h")
include_directories(${FFMPEG_INCLUDE_PATH})

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
endif()
