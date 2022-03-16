// OpenGL plot widget
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef PLOT_WIDGET_H
#define PLOT_WIDGET_H

#include <QOpenGLWidget>

#include <imgui.h>
#include <implot.h>

#include <ros/time.h>

#include <nimbro_topic_transport/SenderStats.h>

#include "scrolling_buffer.h"

namespace nimbro_topic_transport
{

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

	ScrollingBuffer m_buffer;
	std::vector<std::string> m_topics;
};

}

#endif
