#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include "bandwidth_control.h"

namespace nimbro_topic_transport
{

BandwidthControl::BandwidthControl(const uint32_t& bandwidth,
	const uint32_t& release_rate,
	uint32_t (UDPSender::*get_pending_data_size)(),
	void (UDPSender::*send)(),
	UDPSender* object
) : m_BANDWIDTH(bandwidth), m_release_rate(release_rate)
{
	m_send = boost::bind(send, object);
	m_getPendingDataSize = boost::bind(get_pending_data_size, object);
	
	release();
	
	m_bandwidth_releaser_thread = boost::thread(&BandwidthControl::bandwidthReleaserWorker, this);
}

BandwidthControl::~BandwidthControl()
{
	m_bandwidth_releaser_thread.interrupt();
	m_bandwidth_releaser_thread.join();
	
	ROS_INFO("Exitting...");
}

void BandwidthControl::release()
{
	release(0);
}

void BandwidthControl::release(const uint32_t& bytes)
{
	boost::lock_guard<boost::mutex> guard(m_mutex);
	if(!bytes || (m_available + bytes) > m_BANDWIDTH)
	{
		m_available = m_BANDWIDTH;
	}
	else
	{
		m_available += bytes;
	}
	
	ROS_INFO("Release: %d, Available: %d", bytes, m_available);
}

void BandwidthControl::take(const uint32_t& bytes)
{
	uint32_t available;
	
	if(bytes > m_BANDWIDTH)
	{
		ROS_FATAL("The data waiting to be sent is larger than the available bandwidth: %d > %d", bytes, m_BANDWIDTH);
		exit(1);
	}
	
	{
		boost::lock_guard<boost::mutex> guard(m_mutex);
		available = m_available;
	}
	
	while(bytes > available)
	{
		// TODO: sleep how much?
		boost::this_thread::sleep(boost::posix_time::milliseconds(m_release_rate));
		
		{
			boost::lock_guard<boost::mutex> guard(m_mutex);
			available = m_available;
		}
	}
	
	boost::lock_guard<boost::mutex> guard(m_mutex);
	m_available -= bytes;
	
	ROS_INFO("Take: %d, Available: %d", bytes, m_available);
}

void BandwidthControl::send()
{
	uint32_t data_size = m_getPendingDataSize();
	
	ROS_INFO("Sending...");
	
	take(data_size);
	m_send();
}

void BandwidthControl::bandwidthReleaserWorker()
{
	// Every [m_release_rate] milliseconds release [bytes_per_release] bytes
	uint32_t bytes_per_release = (uint32_t)((float) m_BANDWIDTH * ((float) m_release_rate / 1000.));
	
	while(1)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(m_release_rate));
		release(bytes_per_release);
	}
}

}
