#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace nimbro_topic_transport
{

class UDPSender;

class BandwidthControl
{
public:
	BandwidthControl(const uint32_t& bandwidth,
		const uint32_t& release_rate,
		uint32_t (UDPSender::*get_pending_data_size)(),
		void (UDPSender::*send)(),
		UDPSender* object
	);
	
	~BandwidthControl();
	
	void send();
	
private:
	void release();
	void release(const uint32_t& bytes);
	void take(const uint32_t& bytes);
	
	const uint32_t m_BANDWIDTH; // bytes per second
	uint32_t m_available; // bytes
	uint32_t m_release_rate; // milliseconds
	
	boost::mutex m_mutex;
	
	boost::function<void(void)> m_send;
	boost::function<uint32_t(void)> m_getPendingDataSize;
	
	boost::thread m_bandwidth_releaser_thread;
	void bandwidthReleaserWorker();
};

}