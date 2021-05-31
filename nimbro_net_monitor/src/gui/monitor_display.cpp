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

namespace
{
	QString bandwidthToString(double bw)
	{
		for(const char* suffix : {"Bit/s", "KBit/s", "MBit/s", "GBit/s", "TBit/s"})
		{
			if(bw < 1000)
				return QString::number(bw, 'f', 1) + " " + suffix;

			bw /= 1000;
		}

		return "unbelievable!";
	}
}

MonitorDisplay::MonitorDisplay()
{
}

MonitorDisplay::~MonitorDisplay()
{
}

void MonitorDisplay::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
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

	QSharedPointer<QCPAxisTickerDateTime> ticker{new QCPAxisTickerDateTime};
    ticker->setDateTimeFormat("hh:mm:ss");

	for(auto* plot : {m_ui.peerTXPlot, m_ui.peerRXPlot, m_ui.topicTXPlot, m_ui.topicRXPlot})
	{
		plot->xAxis->setTicker(ticker);
		plot->yAxis->setRangeLower(0);

		plot->legend->setVisible(true);
		plot->legend->setIconBorderPen(Qt::NoPen);
		plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
	}

	m_peerRXTotalGraph = m_ui.peerRXPlot->addGraph();
	m_peerRXTotalGraph->setName("Non-ROS / Overhead");
	m_peerRXTotalGraph->setPen(QPen(QColor::fromHsv(0,110,150)));
	m_peerRXTotalGraph->setBrush(QBrush(QColor::fromHsv(0, 0, 50)));

	m_peerTXTotalGraph = m_ui.peerTXPlot->addGraph();
	m_peerTXTotalGraph->setName("Non-ROS / Overhead");
	m_peerTXTotalGraph->setPen(QPen(QColor::fromHsv(0,110,150)));
	m_peerTXTotalGraph->setBrush(QBrush(QColor::fromHsv(0, 0, 50)));

	m_topicRXTotalGraph = m_ui.topicRXPlot->addGraph();
	m_topicRXTotalGraph->setName("Non-ROS / Overhead");
	m_topicRXTotalGraph->setPen(QPen(QColor::fromHsv(0,110,150)));
	m_topicRXTotalGraph->setBrush(QBrush(QColor::fromHsv(0, 0, 50)));

	m_topicTXTotalGraph = m_ui.topicTXPlot->addGraph();
	m_topicTXTotalGraph->setName("Non-ROS / Overhead");
	m_topicTXTotalGraph->setPen(QPen(QColor::fromHsv(0,110,150)));
	m_topicTXTotalGraph->setBrush(QBrush(QColor::fromHsv(0, 0, 50)));

	m_ui.hostComboBox->setModel(&m_hostModel);
	m_ui.interfaceComboBox->setModel(&m_interfaceModel);

	connect(&m_plotTimer, &QTimer::timeout, this, &MonitorDisplay::updatePlot);
	m_plotTimer.start(50);

	connect(m_ui.hostComboBox, &QComboBox::currentTextChanged, this, &MonitorDisplay::reset);
	connect(m_ui.interfaceComboBox, &QComboBox::currentTextChanged, this, &MonitorDisplay::reset);
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
		auto idx = std::lower_bound(
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
			auto it = std::lower_bound(
				m_interfaces.begin(), m_interfaces.end(), iface
			);
			m_interfaces.insert(it, iface);

			m_interfaceModel.setStringList(m_interfaces);
		}

		if(iface != m_ui.interfaceComboBox->currentText())
			continue;

		for(auto* plot : {m_ui.peerTXPlot, m_ui.peerRXPlot, m_ui.topicTXPlot, m_ui.topicRXPlot})
		{
			if(ifaceStats.max_bits_per_second != 0)
				plot->yAxis->setRangeUpper(static_cast<double>(ifaceStats.max_bits_per_second) / 1000 / 1000);
			else
				plot->yAxis->setRangeUpper(100.0);
		}

		for(auto& topic : m_rxTopics)
			topic.second.bandwidth = 0.0;

		for(auto& topic : m_txTopics)
			topic.second.bandwidth = 0.0;

		m_totalRX = ifaceStats.rx_bandwidth / 1000 / 1000;
		m_totalTX = ifaceStats.tx_bandwidth / 1000 / 1000;
		m_ui.duplexLabel->setText(ifaceStats.duplex ? "full" : "half");
		m_ui.speedLabel->setText(bandwidthToString(ifaceStats.max_bits_per_second));
		m_statsTimestamp = msg->header.stamp.toSec();

		for(auto& peerMsg : ifaceStats.peers)
		{
			auto peerName = QString::fromStdString(peerMsg.host);
			auto it = m_peers.find(peerName);
			if(it == m_peers.end())
			{
				ROS_INFO("new peer: %s", qPrintable(peerName));

				std::tie(it, std::ignore) = m_peers.emplace(peerName, Peer());

				Peer& peer = it->second;

				peer.txGraph = m_ui.peerTXPlot->addGraph();
				peer.txGraph->setName(peerName);
				peer.txGraph->setPen(QPen(QColor::fromHsv(0,110,150)));
				peer.txGraph->setBrush(QBrush(QColor::fromHsv(m_hue, BRUSH_SATURATION, BRUSH_VALUE)));

				peer.rxGraph = m_ui.peerRXPlot->addGraph();
				peer.rxGraph->setName(peerName);
				peer.rxGraph->setPen(QPen(QColor::fromHsv(0,110,150)));
				peer.rxGraph->setBrush(QBrush(QColor::fromHsv(m_hue, BRUSH_SATURATION, BRUSH_VALUE)));

				m_hue = (m_hue + 137) % 360;
			}

			auto& peer = it->second;

			peer.tx_bandwidth = 0.0;
			peer.rx_bandwidth = 0.0;
			peer.timestamp = msg->header.stamp.toSec();

			for(auto& nodeMsg : peerMsg.nodes)
			{
				for(auto& conMsg : nodeMsg.connections)
				{
					double bw = static_cast<double>(conMsg.bits_per_second) / 1000 / 1000;

					switch(conMsg.direction)
					{
						case nimbro_net_monitor::ConnectionStats::DIR_IN:
							peer.rx_bandwidth += bw;
							break;
						case nimbro_net_monitor::ConnectionStats::DIR_OUT:
							peer.tx_bandwidth += bw;
							break;
					}

					if(bw > 0.5)
					{
						auto& container = conMsg.direction == nimbro_net_monitor::ConnectionStats::DIR_IN ? m_rxTopics : m_txTopics;

						auto topicName = QString::fromStdString(conMsg.topic);
						auto it = container.find(topicName);
						if(it == container.end())
						{
							std::tie(it, std::ignore) = container.emplace(topicName, TopicInfo());

							TopicInfo& topic = it->second;

							topic.graph = m_ui.topicRXPlot->addGraph();
							topic.graph->setName(topicName);
							topic.graph->setPen(QPen(QColor::fromHsv(0,110,150)));
							topic.graph->setBrush(QBrush(QColor::fromHsv(m_hue, BRUSH_SATURATION, BRUSH_VALUE)));

							m_hue = (m_hue + 137) % 360;
						}

						auto& topicInfo = it->second;

						topicInfo.bandwidth += bw;
						topicInfo.timeLastSeen = ros::SteadyTime::now();
						topicInfo.timestamp = msg->header.stamp.toSec();
					}
				}
			}

// 			ROS_INFO("peer %s: %f/%f", qPrintable(peerName), peer.tx_bandwidth, peer.rx_bandwidth);
		}
	}

	// Peer TX
	{
		double plotValue = 0;
		QCPGraph* prevGraph = nullptr;
		m_ui.peerTXPlot->legend->clearItems();
		m_peerTXTotalGraph->addToLegend(m_ui.peerTXPlot->legend);

		for(auto it = m_peers.rbegin(); it != m_peers.rend(); ++it)
			it->second.txGraph->addToLegend(m_ui.peerTXPlot->legend);

		for(auto& pair : m_peers)
		{
			auto& peer = pair.second;

			plotValue += peer.tx_bandwidth;

			if(prevGraph)
				peer.txGraph->setChannelFillGraph(prevGraph);

			prevGraph = peer.txGraph;

			peer.txGraph->addData(peer.timestamp, plotValue);

			peer.txGraph->data()->removeBefore(peer.timestamp - WINDOW_SIZE*2);
		}

		if(prevGraph)
			m_peerTXTotalGraph->setChannelFillGraph(prevGraph);

		m_peerTXTotalGraph->addData(m_statsTimestamp, m_totalTX);
		m_peerTXTotalGraph->data()->removeBefore(m_statsTimestamp - WINDOW_SIZE*2);
	}

	// Peer RX
	{
		double plotValue = 0;
		QCPGraph* prevGraph = nullptr;

		m_ui.peerRXPlot->legend->clearItems();
		m_peerRXTotalGraph->addToLegend(m_ui.peerRXPlot->legend);

		for(auto it = m_peers.rbegin(); it != m_peers.rend(); ++it)
			it->second.rxGraph->addToLegend(m_ui.peerRXPlot->legend);

		for(auto& pair : m_peers)
		{
			auto& peer = pair.second;

			plotValue += peer.rx_bandwidth;

			if(prevGraph)
				peer.rxGraph->setChannelFillGraph(prevGraph);

			prevGraph = peer.rxGraph;

			peer.rxGraph->addData(peer.timestamp, plotValue);

			peer.rxGraph->data()->removeBefore(peer.timestamp - WINDOW_SIZE*2);
		}

		if(prevGraph)
			m_peerRXTotalGraph->setChannelFillGraph(prevGraph);

		m_peerRXTotalGraph->addData(m_statsTimestamp, m_totalRX);
		m_peerRXTotalGraph->data()->removeBefore(m_statsTimestamp - WINDOW_SIZE*2);
	}

	// Topic TX
	{
		double plotValue = 0;
		QCPGraph* prevGraph = nullptr;

		m_ui.topicTXPlot->legend->clearItems();
		m_topicTXTotalGraph->addToLegend(m_ui.topicTXPlot->legend);

		for(auto it = m_txTopics.rbegin(); it != m_txTopics.rend(); ++it)
			it->second.graph->addToLegend(m_ui.topicTXPlot->legend);

		for(auto& pair : m_txTopics)
		{
			auto& topic = pair.second;

			plotValue += topic.bandwidth;

			if(prevGraph)
				topic.graph->setChannelFillGraph(prevGraph);

			prevGraph = topic.graph;

			topic.graph->addData(topic.timestamp, plotValue);
			topic.graph->data()->removeBefore(topic.timestamp - WINDOW_SIZE*2);
		}

		if(prevGraph)
			m_topicTXTotalGraph->setChannelFillGraph(prevGraph);

		m_topicTXTotalGraph->addData(m_statsTimestamp, m_totalTX);
		m_topicTXTotalGraph->data()->removeBefore(m_statsTimestamp - WINDOW_SIZE*2);
	}

	// Topic RX
	{
		double plotValue = 0;
		QCPGraph* prevGraph = nullptr;

		m_ui.topicRXPlot->legend->clearItems();
		m_topicRXTotalGraph->addToLegend(m_ui.topicRXPlot->legend);

		for(auto it = m_rxTopics.rbegin(); it != m_rxTopics.rend(); ++it)
			it->second.graph->addToLegend(m_ui.topicRXPlot->legend);

		for(auto& pair : m_rxTopics)
		{
			auto& topic = pair.second;

			plotValue += topic.bandwidth;

			if(prevGraph)
				topic.graph->setChannelFillGraph(prevGraph);

			prevGraph = topic.graph;

			topic.graph->addData(topic.timestamp, plotValue);
			topic.graph->data()->removeBefore(topic.timestamp - WINDOW_SIZE*2);
		}

		if(prevGraph)
			m_topicRXTotalGraph->setChannelFillGraph(prevGraph);

		m_topicRXTotalGraph->addData(m_statsTimestamp, m_totalRX);
		m_topicRXTotalGraph->data()->removeBefore(m_statsTimestamp - WINDOW_SIZE*2);
	}
}

void MonitorDisplay::updatePlot()
{
	for(auto* plot : {m_ui.peerTXPlot, m_ui.peerRXPlot, m_ui.topicTXPlot, m_ui.topicRXPlot})
	{
		plot->xAxis->setRange(ros::Time::now().toSec() + 0.25, WINDOW_SIZE, Qt::AlignRight);
		plot->replot();
	}
}

void MonitorDisplay::reset()
{
	m_peers.clear();
	m_rxTopics.clear();
	m_txTopics.clear();
	m_ui.speedLabel->setText("-");
	m_ui.duplexLabel->setText("-");

	m_topicRXTotalGraph->data()->clear();
	m_topicTXTotalGraph->data()->clear();
	m_peerRXTotalGraph->data()->clear();
	m_peerTXTotalGraph->data()->clear();

	m_totalRX = 0;
	m_totalTX = 0;
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_net_monitor::MonitorDisplay, rqt_gui_cpp::Plugin)
