// H264 sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C"
{
#include <x264.h>
}

#include "rgb_to_yuv420.h"

ros::Time g_lastImageTime(0);
ros::Duration g_minTimeBetweenImages;

int g_width;

std::vector<uint8_t> g_inBuf;
x264_t* g_encoder = 0;
int g_encoderConfiguredHeight;

x264_picture_t g_inputPicture;
x264_picture_t g_outPicture;

ros::Publisher g_pub;

double g_crf;

void handleImage(const sensor_msgs::ImageConstPtr& img)
{
	ros::Time now = ros::Time::now();
	if(now - g_lastImageTime < g_minTimeBetweenImages)
		return;
	g_lastImageTime = now;

	ros::Time start = ros::Time::now();

	cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(img, "bgr8");

	int height = g_width * cvImg->image.rows / cvImg->image.cols;

	cv::Mat resized;
	cv::resize(cvImg->image, resized, cv::Size(g_width, height), CV_INTER_AREA);

	if(!g_encoder)
	{
		x264_param_t params;
		x264_param_default(&params);
		x264_param_apply_profile(&params, "high");
		x264_param_default_preset(&params, "ultrafast", "zerolatency");

		params.i_width = g_width;
		params.i_height = height;
		params.b_repeat_headers = 1;
		params.b_intra_refresh = 1;
		params.i_fps_num = 1;
		params.i_fps_den = 10;
		params.i_frame_reference = 1;
		params.i_keyint_max = 20;
		params.i_bframe = 0;
		params.b_open_gop = 0;
	// 	params.rc.i_rc_method = X264_RC_CRF;
	// // 	params.rc.i_qp_min = params.rc.i_qp_max = 47;
	// 	params.rc.i_vbv_buffer_size = 6;
	// 	params.rc.i_vbv_max_bitrate = 6000;
	// 	params.rc.i_bitrate = 6;
		params.rc.i_rc_method = X264_RC_CRF;
		params.rc.f_rf_constant = g_crf;
		params.rc.i_vbv_buffer_size = 1000;
		params.rc.i_vbv_max_bitrate = 1000;
		params.rc.i_bitrate = 1000;
		params.i_threads = 4;

		g_encoder = x264_encoder_open(&params);

		x264_picture_init(&g_inputPicture);
		x264_picture_init(&g_outPicture);

		g_encoderConfiguredHeight = height;

		g_inBuf.resize(g_width*height + g_width*height/2);
	}
	else if(height != g_encoderConfiguredHeight)
	{
		ROS_ERROR("Image dimensions changed!");
		throw std::runtime_error("Image dimensions changed");
	}

	RGB_to_YUV420(resized.data, g_inBuf.data(), g_width, height);

	g_inputPicture.img.plane[0] = g_inBuf.data();
	g_inputPicture.img.plane[1] = g_inBuf.data() + g_width*height;
	g_inputPicture.img.plane[2] = g_inBuf.data() + g_width*height + g_width*height/4;

	g_inputPicture.img.i_stride[0] = g_width;
	g_inputPicture.img.i_stride[1] = g_width/2;
	g_inputPicture.img.i_stride[2] = g_width/2;

	g_inputPicture.img.i_csp = X264_CSP_I420;
	g_inputPicture.img.i_plane = 3;

	x264_nal_t* nals;
	int numNals;

	x264_encoder_encode(g_encoder, &nals, &numNals, &g_inputPicture, &g_outPicture);

	std::size_t size = 0;
	for(int i = 0; i < numNals; ++i)
		size += nals[i].i_payload;

	sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage);

	msg->header = img->header;
	msg->format = "h264";

	msg->data.resize(size);
	unsigned int off = 0;

	for(int i = 0; i < numNals; ++i)
	{
		memcpy(msg->data.data() + off, nals[i].p_payload, nals[i].i_payload);
		off += nals[i].i_payload;
	}

	g_pub.publish(msg);
	ROS_DEBUG("took %f", (ros::Time::now() - start).toSec());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cam_sender");

	ros::NodeHandle nh("~");

	image_transport::ImageTransport it(nh);

	nh.param("width", g_width, 640);

	double rate;
	nh.param("rate", rate, 60.0);

	nh.param("crf", g_crf, 30.0);

	g_minTimeBetweenImages = ros::Duration(1.0 / rate);

	g_pub = nh.advertise<sensor_msgs::CompressedImage>("encoded", 1);

	image_transport::Subscriber sub = it.subscribe("image", 1, &handleImage);

	ros::spin();

	return 0;
}
