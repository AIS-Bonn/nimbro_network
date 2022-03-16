// Circular buffer for plotting
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "scrolling_buffer.h"

#include <cstring>

namespace nimbro_topic_transport
{

ScrollingBuffer::ScrollingBuffer()
{
}

void ScrollingBuffer::reset(std::size_t rows)
{
	m_storage.clear();
	m_storage.resize(rows * BUFFER_SIZE, 0.0f);
	m_rows = rows;
	m_size = 0;
	m_offset = 0;
	m_max = 0.0f;
}

void ScrollingBuffer::addRow(unsigned int row)
{
	// Add new row in storage with zeros
	m_storage.insert(m_storage.begin() + row*BUFFER_SIZE, BUFFER_SIZE, 0.0f);

	// Add new row in storageAcc
	m_storageAcc.insert(m_storageAcc.begin() + row*BUFFER_SIZE, BUFFER_SIZE, 0.0f);

	// And initialize from row before
	if(row != 0)
	{
		std::copy(
			m_storageAcc.begin() + (row-1)*BUFFER_SIZE,
			m_storageAcc.begin() + row*BUFFER_SIZE,
			m_storageAcc.begin() + row*BUFFER_SIZE
		);
	}

	m_rows++;
}

void ScrollingBuffer::push_back(float time, float* data)
{
	if(m_size < BUFFER_SIZE)
	{
		float accum = 0.0f;
		for(std::size_t row = 0; row < m_rows; ++row)
		{
			m_storage[row * BUFFER_SIZE + m_size] = data[row];

			accum += data[row];
			m_storageAcc[row * BUFFER_SIZE + m_size] = accum;
		}
		m_time[m_size] = time;

		m_size++;
	}
	else
	{
		float accum = 0.0f;
		for(std::size_t row = 0; row < m_rows; ++row)
		{
			m_storage[row * BUFFER_SIZE + m_offset] = data[row];

			accum += data[row];
			m_storageAcc[row * BUFFER_SIZE + m_offset] = accum;
		}
		m_time[m_offset] = time;

		m_offset = (m_offset + 1) % BUFFER_SIZE;
	}

	// Update maximum
	m_max = 0.0f;
	for(std::size_t time = 0; time < BUFFER_SIZE; ++time)
	{
		m_max = std::max(m_max, m_storageAcc[(m_rows-1)*BUFFER_SIZE + time]);
	}
}

}
