// Distributes incoming messages among worker threads for compression + FEC
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "thread_pool.h"

namespace nimbro_topic_transport
{

ThreadPool::ThreadPool()
{
	unsigned int threadCount = std::thread::hardware_concurrency();
	if(threadCount == 0)
		threadCount = 2;

	ROS_DEBUG("Starting %u worker threads", threadCount);
	for(unsigned int i = 0; i < threadCount; ++i)
		m_threads.emplace_back(std::bind(&ThreadPool::work, this));
}

ThreadPool::~ThreadPool()
{
	// Stop the worker threads
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_shouldExit = true;
	}
	m_cond.notify_all();

	for(auto& thread : m_threads)
		thread.join();
}

void ThreadPool::work()
{
	while(1)
	{
		Message::ConstPtr jobData;
		WorkBuffer::Ptr workBuffer;

		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_cond.wait(lock, [&]() -> bool {
				if(m_shouldExit)
					return true;

				// Look for open jobs. We try to be fair between topics here, so that
				// one high-bandwidth topic cannot starve the other topics.

				for(unsigned int i = 0; i < m_workBuffers.size(); ++i)
				{
					unsigned int idx = (m_workCheckIdx + i) % m_workBuffers.size();
					if(!m_workBuffers[idx]->jobs.empty())
					{
						// Take it!
						workBuffer = m_workBuffers[idx];
						jobData = std::move(workBuffer->jobs.front());
						workBuffer->jobs.pop_front();

						// Let the next thread start searching after this element,
						// this guarantees that this topic cannot starve the others.
						m_workCheckIdx = (idx + 1) % m_workBuffers.size();
						return true;
					}
				}

				return false;
			});
		}

		if(m_shouldExit)
			break;

		if(jobData)
		{
			// Process the message
			workBuffer->callback(jobData);
		}
	}
}

ThreadPool::Callback ThreadPool::createInputHandler(const Callback& cb)
{
	std::unique_lock<std::mutex> lock(m_mutex);

	auto wb = std::make_shared<WorkBuffer>();
	wb->callback = cb;

	m_workBuffers.push_back(wb);
	return std::bind(&ThreadPool::handleInput, this, wb, std::placeholders::_1);
}

void ThreadPool::handleInput(const WorkBuffer::Ptr& wb, const Message::ConstPtr& msg)
{
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		wb->jobs.push_back(msg);

		if(wb->jobs.size() > 5)
		{
			ROS_WARN_THROTTLE(1.0, "Dropping job from ThreadPool. Is the system overloaded?");
			wb->jobs.pop_front();
		}
	}
	m_cond.notify_one();
}

}

