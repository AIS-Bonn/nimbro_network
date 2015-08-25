// Bandwidth display for transferred topics
// Author: Sebastian Schüller

#include "bandwidth_gui.h"
#include "contrib/qcustomplot.h"

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

namespace nimbro_topic_transport
{

BandwidthGui::BandwidthGui()
{}

BandwidthGui::~BandwidthGui()
{}

void BandwidthGui::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_plot = new QCustomPlot();
	ctx.addWidget(m_plot);

	connect(
		this, SIGNAL(senderStatsReceived(nimbro_topic_transport::SenderStatsConstPtr)),
		this, SLOT(handleSenderStats(nimbro_topic_transport::SenderStatsConstPtr)),
		Qt::QueuedConnection
	);

	m_sub_senderStats = getPrivateNodeHandle().subscribe(
		"/network/sender_stats", 1, &BandwidthGui::senderStatsReceived, this
	);
	m_plot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
	m_plot->xAxis->setDateTimeFormat("hh:mm:ss");
	m_plot->xAxis->setAutoTickStep(false);
	m_plot->xAxis->setTickStep(2);
	m_plot->axisRect()->setupFullAxesBox();
  

}

void BandwidthGui::shutdownPlugin()
{
	m_sub_senderStats.shutdown();
}

void BandwidthGui::handleSenderStats(const nimbro_topic_transport::SenderStatsConstPtr& msg)
{
	for(const auto& topic : msg->topics)
	{
		if(m_bandwidths.find(topic.name) == m_bandwidths.end())
		{
			// Add and configure new graph to plot
			m_bandwidths[topic.name].graph = m_plot->addGraph();
		}
		m_bandwidths[topic.name].value = topic.bandwidth;
		m_bandwidths[topic.name].timestamp = msg->header.stamp.toSec();
	}

	updatePlot();

}

void BandwidthGui::updatePlot()
{
	double plotValue = 0;
	for(const auto& bw : m_bandwidths)
	{
		const auto& gv = bw.second;
		plotValue += gv.value;
		gv.graph->addData(gv.timestamp, plotValue);
		gv.graph->rescaleValueAxis();
	}
	m_plot->xAxis->setRange(ros::Time::now().toSec() + 0.25, 8, Qt::AlignRight);
	m_plot->replot();
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_topic_transport::BandwidthGui, rqt_gui_cpp::Plugin)
