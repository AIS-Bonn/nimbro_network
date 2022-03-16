// Display for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef TOPIC_GUI_H
#define TOPIC_GUI_H

#include <rqt_gui_cpp/plugin.h>

#include <nimbro_topic_transport/SenderStats.h>

#include <ros/subscriber.h>

#include "ui_topic_gui.h"

namespace nimbro_topic_transport
{

class TopicGUI : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	TopicGUI();

	void initPlugin(qt_gui_cpp::PluginContext& ctx) override;
	void shutdownPlugin() override;

Q_SIGNALS:
	void dataReceived(const nimbro_topic_transport::SenderStatsConstPtr& msg);

private Q_SLOTS:
	void refreshTopics();
	void selectTopic(const QString& topic);
	void handleTopicChange();

private:
	using Settings = qt_gui_cpp::Settings;
	void saveSettings(Settings& pluginSettings, Settings& instanceSettings) const override;
	void restoreSettings(const Settings& pluginSettings, const Settings& instanceSettings) override;

	ros::Subscriber m_sub;

	Ui_TopicGUI m_ui;
};

}

#endif
