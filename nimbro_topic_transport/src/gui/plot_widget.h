// OpenGL plot widget
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef PLOT_WIDGET_H
#define PLOT_WIDGET_H

#include <QOpenGLWidget>

#include <imgui.h>
#include <implot.h>

#include <ros/time.h>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

constexpr float PLOT_HISTORY_SECS = 20.0f;
constexpr float PLOT_DATA_RATE_HZ = 10;
constexpr int PLOT_BUFFER_SIZE = 1.2f * PLOT_HISTORY_SECS * PLOT_DATA_RATE_HZ;

class ScrollingBuffer
{
public:
	ScrollingBuffer(std::size_t initialSize = 0)
	{
		m_size = initialSize;
	}

	void push_back(float data)
	{
		if(m_size < PLOT_BUFFER_SIZE)
		{
			m_data[m_size] = data;
			m_size++;
		}
		else
		{
			m_data[m_offset] = data;
			m_offset = (m_offset + 1) % PLOT_BUFFER_SIZE;
		}
	}

	const float* data() const
	{ return m_data.data(); }

	std::size_t size() const
	{ return m_size; }

	std::size_t offset() const
	{ return m_offset; }

	const float& back() const
	{
		if(m_size != PLOT_BUFFER_SIZE)
			return m_data[m_size-1];
		else
			return m_data[(m_offset + PLOT_BUFFER_SIZE - 1) % PLOT_BUFFER_SIZE];
	}
private:
	std::array<float, PLOT_BUFFER_SIZE> m_data;
	std::size_t m_size = 0;
	std::size_t m_offset = 0;
};

class PlotWidget : public QOpenGLWidget
{
public:
	explicit PlotWidget(QWidget* parent = nullptr);
	~PlotWidget();

	void initializeGL() override;
	void resizeGL(int w, int h) override;

	void paintGL() override;

	void integrateData(const nimbro_topic_transport::SenderStatsConstPtr& msg);
	void clear();

private:
	ImGuiContext* m_imgui = {};
	ImGuiIO* m_io = {};
	ImPlotContext* m_implot = {};
	ImFont* m_font_small = {};

	ros::Time m_plotTimeBase = ros::Time::now();

	struct Topic
	{
		Topic(std::size_t initialBufSize)
		 : bandwidth{initialBufSize}
		{}

		ScrollingBuffer bandwidth;
	};

	ScrollingBuffer m_timeBuffer;
	std::map<std::string, Topic> m_topics;
};

}

#endif
