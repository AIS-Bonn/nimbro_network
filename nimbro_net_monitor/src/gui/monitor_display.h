// Bandwidth display for transferred topics
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef MONITOR_DISPLAY_H
#define MONITOR_DISPLAY_H

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <nimbro_net_monitor/NetworkStats.h>

#include <map>
#include <memory>

#include <QTimer>

#include "ui_monitor_display.h"

class Ui_MonitorDisplay;

namespace nimbro_net_monitor
{

class MonitorDisplay : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	MonitorDisplay();
	virtual ~MonitorDisplay();

	virtual void initPlugin(qt_gui_cpp::PluginContext & ) override;
	virtual void shutdownPlugin() override;
// 	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
// 	virtual void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;

Q_SIGNALS:
	// Bounce SenderStats msg from ros thread to GUI thread
	void statsReceived(const nimbro_net_monitor::NetworkStatsConstPtr& msg);

private Q_SLOTS:
	void handleStats(const nimbro_net_monitor::NetworkStatsConstPtr& msg);

// 	void handleClickedLegend(QCPLegend *legend, QCPAbstractLegendItem *item, QMouseEvent *event);

	void updatePlot();

	void reset();

private:
	struct Peer
	{
		Peer() = default;
		Peer(Peer&&) = default;
		Peer(const Peer&) = delete;

		~Peer()
		{
			if(rxGraph)
				rxGraph->parentPlot()->removeGraph(rxGraph);

			if(txGraph)
				txGraph->parentPlot()->removeGraph(rxGraph);
		}

		Peer& operator=(Peer&&) = default;
		Peer& operator=(const Peer&) = delete;

		QCPGraph* rxGraph = nullptr;
		QCPGraph* txGraph = nullptr;
		double timestamp;
		double tx_bandwidth; // MBit/s
		double rx_bandwidth; // MBit/s
	};

	struct TopicInfo
	{
		TopicInfo() = default;
		TopicInfo(TopicInfo&&) = default;
		TopicInfo(const TopicInfo&) = delete;

		~TopicInfo()
		{
			if(graph)
				graph->parentPlot()->removeGraph(graph);
		}

		TopicInfo& operator=(TopicInfo&&) = default;
		TopicInfo& operator=(const TopicInfo&) = delete;

		QCPGraph* graph = nullptr;

		ros::SteadyTime timeLastSeen;
		double timestamp;
		double bandwidth; // MBit/s
	};

	ros::Subscriber m_sub_stats;

	Ui_MonitorDisplay m_ui;

	QStringList m_hosts;
	QStringListModel m_hostModel;

	QStringList m_interfaces;
	QStringListModel m_interfaceModel;

	std::map<QString, Peer> m_peers;

	QTimer m_plotTimer;

	QCPGraph* m_peerRXTotalGraph = nullptr;
	QCPGraph* m_peerTXTotalGraph = nullptr;

	QCPGraph* m_topicRXTotalGraph = nullptr;
	QCPGraph* m_topicTXTotalGraph = nullptr;

	double m_statsTimestamp = 0.0;
	double m_totalTX = 0.0; // MBit/s
	double m_totalRX = 0.0; // MBit/s

	int m_hue = 0;

	std::map<QString, TopicInfo> m_txTopics;
	std::map<QString, TopicInfo> m_rxTopics;
};

}
#endif
