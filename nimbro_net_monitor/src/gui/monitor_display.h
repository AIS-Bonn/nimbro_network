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

private:
	struct Peer
	{
		QCPGraph* graph;
		double timestamp;
		double tx_bandwidth; // MBit/s
		double rx_bandwidth; // MBit/s
	};

	ros::Subscriber m_sub_stats;

	Ui_MonitorDisplay m_ui;

	QStringList m_hosts;
	QStringListModel m_hostModel;

	QStringList m_interfaces;
	QStringListModel m_interfaceModel;

	QMap<QString, Peer> m_peers;

	QTimer m_plotTimer;

	int m_hue = 0;
};

}
#endif
