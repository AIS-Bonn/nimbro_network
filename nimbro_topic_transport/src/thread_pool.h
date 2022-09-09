// Distributes incoming messages among worker threads for compression + FEC
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_THREAD_POOL_H
#define TT_THREAD_POOL_H

#include "message.h"

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace nimbro_topic_transport
{

class ThreadPool
{
public:
	typedef std::function<void(const Message::ConstPtr& msg)> Callback;

	explicit ThreadPool(int threadCount = -1);
	~ThreadPool();

	ThreadPool(const ThreadPool&) = delete;
	ThreadPool& operator=(const ThreadPool&) = delete;

	Callback createInputHandler(const Callback& cb);
private:
	void work();

	//! @name Threading
	//@{
	std::vector<std::thread> m_threads;
	bool m_shouldExit = false;

	std::mutex m_mutex;
	std::condition_variable m_cond;
	//@}

	//! @name Work queues
	//@{
	struct WorkBuffer
	{
		typedef std::shared_ptr<WorkBuffer> Ptr;
		typedef std::shared_ptr<const WorkBuffer> ConstPtr;

		std::deque<Message::ConstPtr> jobs;
		Callback callback;
	};

	//! Called from input (usually Subscriber)
	void handleInput(const WorkBuffer::Ptr& wb, const Message::ConstPtr& msg);

	std::vector<WorkBuffer::Ptr> m_workBuffers;
	unsigned int m_workCheckIdx = 0;
	//@}
};

}

#endif

