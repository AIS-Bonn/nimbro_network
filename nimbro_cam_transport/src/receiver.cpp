// H264 receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <image_transport/image_transport.h>

#include <sensor_msgs/CompressedImage.h>

ros::Publisher g_pub;
AVCodecContext* g_codec = 0;
SwsContext* g_sws = 0;

const char* averror(int code)
{
	static char buf[256];
	av_strerror(code, buf, sizeof(buf));
	return buf;
}

void handleImage(const sensor_msgs::CompressedImageConstPtr& img)
{
	AVPacket packet;
	av_init_packet(&packet);
	packet.data = const_cast<uint8_t*>(img->data.data());
	packet.size = img->data.size();
	packet.pts = AV_NOPTS_VALUE;
	packet.dts = AV_NOPTS_VALUE;

	AVFrame frame;
	memset(&frame, 0, sizeof(frame));

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57,48,101)
	// Input packet to the encoder
	int ret = avcodec_send_packet(g_codec, &packet);
	if(ret < 0)
	{
		ROS_ERROR("Could not send packet to decoder: %s", averror(ret));
		return;
	}

	// Try to retrieve output frame
	ret = avcodec_receive_frame(g_codec, &frame);
	if(ret == AVERROR(EAGAIN))
		return;

	if(ret < 0)
	{
		ROS_ERROR("Could not retrieve frame from decoder: %s", averror(ret));
		return;
	}
#else
	// old API

	int gotPicture = 0;
	if(avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
	{
		ROS_ERROR("avcodec_decode_video2 error");
		return;
	}

	if(!gotPicture)
		return;
#endif

	g_sws = sws_getCachedContext(
		g_sws,
		frame.width, frame.height, AV_PIX_FMT_YUV420P,
		frame.width, frame.height, AV_PIX_FMT_RGB24,
		0, 0, 0, 0
	);

	sensor_msgs::ImagePtr out_img(new sensor_msgs::Image);

	out_img->encoding = "rgb8";
	out_img->data.resize(frame.width * frame.height * 3);
	out_img->step = frame.width * 3;
	out_img->width = frame.width;
	out_img->height = frame.height;
	out_img->header.frame_id = "cam";
	out_img->header.stamp = ros::Time::now(); // FIXME

	uint8_t* destData[1] = {out_img->data.data()};
	int linesize[1] = {(int)out_img->step};

	sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
		destData, linesize);

	g_pub.publish(out_img);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cam_receiver");
	
	ros::NodeHandle nh("~");

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    avcodec_register_all();
#endif

	av_log_set_level(AV_LOG_QUIET);

	AVCodec* decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
	if(!decoder)
		throw std::runtime_error("H264 decoding not supported in this build of ffmpeg");

	g_codec = avcodec_alloc_context3(decoder);

	g_codec->flags |= AV_CODEC_FLAG_LOW_DELAY;

#ifdef AV_CODEC_FLAG2_SHOW_ALL
	g_codec->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
#else
#warning This version of FFMPEG does not offer AV_CODEC_FLAG2_SHOW_ALL. Consider upgrading your FFMPEG.
#endif

	g_codec->thread_type = 0;

	if(avcodec_open2(g_codec, decoder, 0) != 0)
		throw std::runtime_error("Could not open decoder");

	g_pub = nh.advertise<sensor_msgs::Image>("image", 1);
	
	ros::Subscriber sub = nh.subscribe("encoded", 25, &handleImage);
	
	ros::spin();
	
	return 0;
}
