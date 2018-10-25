// Bandwidth display for transferred topics
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "monitor_display.h"

#include <ros/node_handle.h>

#include <pluginlib/class_list_macros.h>

Q_DECLARE_METATYPE(nimbro_net_monitor::NetworkStatsConstPtr)

static const int WINDOW_SIZE = 64;
static const int BRUSH_VALUE = 180;
static const int BRUSH_SATURATION = 150;

namespace nimbro_net_monitor
{

MonitorDisplay::MonitorDisplay()
{
}

MonitorDisplay::~MonitorDisplay()
{
}

void MonitorDisplay::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	ROS_INFO("initPlugin()");
	QWidget* w = new QWidget();
	m_ui.setupUi(w);

	ctx.addWidget(w);

	qRegisterMetaType<nimbro_net_monitor::NetworkStatsConstPtr>();

	connect(
		this, &MonitorDisplay::statsReceived,
		this, &MonitorDisplay::handleStats,
		Qt::QueuedConnection
	);

	m_sub_stats = getPrivateNodeHandle().subscribe(
		"/network/monitor", 1,
		&MonitorDisplay::statsReceived, this
	);

	m_ui.plot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
	m_ui.plot->xAxis->setDateTimeFormat("hh:mm:ss");
	m_ui.plot->xAxis->setAutoTickStep(true);

	m_ui.plot->yAxis->setRangeLower(0);

	m_ui.hostComboBox->setModel(&m_hostModel);
	m_ui.interfaceComboBox->setModel(&m_interfaceModel);

	connect(&m_plotTimer, &QTimer::timeout, this, &MonitorDisplay::updatePlot);
	m_plotTimer.start(50);
}

void MonitorDisplay::shutdownPlugin()
{
	m_sub_stats.shutdown();
	m_plotTimer.stop();
}

void nimbro_net_monitor::MonitorDisplay::handleStats(const nimbro_net_monitor::NetworkStatsConstPtr& msg)
{
	auto host = QString::fromStdString(msg->host);

	if(!m_hosts.contains(host))
	{
		ROS_INFO("new host: %s", qPrintable(host));
		auto idx = qLowerBound(
			m_hosts.begin(), m_hosts.end(), host
		);
		m_hosts.insert(idx, host);

		m_hostModel.setStringList(m_hosts);
	}

	if(host != m_ui.hostComboBox->currentText())
		return;

	for(auto& ifaceStats : msg->interfaces)
	{
		auto iface = QString::fromStdString(ifaceStats.interface_name);
		if(!m_interfaces.contains(iface))
		{
			ROS_INFO("new interface: %s", qPrintable(iface));
			auto it = qLowerBound(
				m_interfaces.begin(), m_interfaces.end(), iface
			);
			m_interfaces.insert(it, iface);

			m_interfaceModel.setStringList(m_interfaces);
		}

		if(iface != m_ui.interfaceComboBox->currentText())
			continue;

		if(ifaceStats.max_bits_per_second != 0)
		{
			m_ui.plot->yAxis->setRangeUpper(static_cast<double>(ifaceStats.max_bits_per_second) / 1000 / 1000);
		}
		else
			m_ui.plot->yAxis->setRangeUpper(100.0);

		for(auto& peerMsg : ifaceStats.peers)
		{
			auto peerName = QString::fromStdString(peerMsg.host);
			auto it = m_peers.find(peerName);
			if(it == m_peers.end())
			{
				ROS_INFO("new peer: %s", qPrintable(peerName));
				Peer peer;
				peer.graph = m_ui.plot->addGraph();

				peer.graph->setName(peerName);
				peer.graph->setPen(QPen(QColor::fromHsv(0,110,150)));
				peer.graph->setBrush(QBrush(QColor::fromHsv(m_hue, BRUSH_SATURATION, BRUSH_VALUE)));
				m_hue = (m_hue + 137) % 360;

				it = m_peers.insert(peerName, peer);
			}

			auto& peer = *it;

			peer.tx_bandwidth = 0.0;
			peer.rx_bandwidth = 0.0;
			peer.timestamp = msg->header.stamp.toSec();

			for(auto& nodeMsg : peerMsg.nodes)
			{
				for(auto& conMsg : nodeMsg.connections)
				{
					switch(conMsg.direction)
					{
						case nimbro_net_monitor::ConnectionStats::DIR_IN:
							peer.rx_bandwidth += static_cast<double>(conMsg.bits_per_second) / 1000 / 1000;
							break;
						case nimbro_net_monitor::ConnectionStats::DIR_OUT:
							peer.tx_bandwidth += static_cast<double>(conMsg.bits_per_second) / 1000 / 1000;
							break;
					}
				}
			}

			ROS_INFO("peer %s: %f/%f", qPrintable(peerName), peer.tx_bandwidth, peer.rx_bandwidth);
		}
	}
}

void MonitorDisplay::updatePlot()
{
	double plotValue = 0;

	QCPGraph* prevGraph = nullptr;

	for(auto& peer : m_peers)
	{
		plotValue += peer.rx_bandwidth;

		if(prevGraph)
			peer.graph->setChannelFillGraph(prevGraph);

		prevGraph = peer.graph;

		peer.graph->addData(peer.timestamp, plotValue);

		peer.graph->removeDataBefore(peer.timestamp - WINDOW_SIZE);
	}

	m_ui.plot->xAxis->setRange(ros::Time::now().toSec() + 0.25, WINDOW_SIZE, Qt::AlignRight);
	m_ui.plot->replot();
}


}

PLUGINLIB_EXPORT_CLASS(nimbro_net_monitor::MonitorDisplay, rqt_gui_cpp::Plugin)
