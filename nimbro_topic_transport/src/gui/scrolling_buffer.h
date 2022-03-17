// Circular buffer for plotting
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef SCROLLING_BUFFER_H
#define SCROLLING_BUFFER_H

#include <vector>
#include <array>

namespace nimbro_topic_transport
{

class ScrollingBuffer
{
public:
	static constexpr float HISTORY_SECS = 20.0f;
	static constexpr float DATA_RATE_HZ = 10;
	static constexpr int BUFFER_SIZE = 1.2f * HISTORY_SECS * DATA_RATE_HZ;

	ScrollingBuffer();

	const float* timeData() const
	{ return m_time.data(); }

	const float* rowData(unsigned int row) const
	{ return m_storage.data() + row*BUFFER_SIZE; }

	const float* rowAccData(unsigned int row) const
	{ return m_storageAcc.data() + row*BUFFER_SIZE; }

	std::size_t rows() const
	{ return m_rows; }

	std::size_t size() const
	{ return m_size; }

	//! Time offset where we should start to read (oldest entry)
	std::size_t offset() const
	{ return m_offset; }

	float maximum() const
	{ return m_max; }

	float lastTime() const
	{
		if(m_size != BUFFER_SIZE)
            return m_time[m_size-1];
        else
            return m_time[(m_offset + BUFFER_SIZE - 1) % BUFFER_SIZE];
	}

	void reset(std::size_t rows);
	void addRow(unsigned int row);
	void push_back(float time, float* data);
private:
	std::array<float, BUFFER_SIZE> m_time;
	std::vector<float> m_storage;
	std::vector<float> m_storageAcc;

	std::size_t m_size = 0;
	std::size_t m_offset = 0;
	std::size_t m_rows = 0;

	float m_max = 0.0f;
};

}

#endif
