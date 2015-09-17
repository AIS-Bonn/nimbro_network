// Bandwidth display for transferred topics
// Author: Sebastian Schüller

#ifndef BANDWIDTH_GUI
#define BANDWIDTH_GUI

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <nimbro_topic_transport/SenderStats.h>
#include <map>

#include <QTimer>
#include <QDialog>

class QCustomPlot;
class QCPGraph;
class QCPLegend;
class QCPAbstractLegendItem;
class QMouseEvent;
class QComboBox;
class QTextEdit;

typedef std::map<std::string, std::vector<std::string> > GroupMap;

namespace nimbro_topic_transport
{


class BandwidthGui : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	BandwidthGui();
	virtual ~BandwidthGui();

	virtual void initPlugin(qt_gui_cpp::PluginContext & ) override;
	virtual void shutdownPlugin() override;
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
	virtual void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;

Q_SIGNALS:
	// Bounce SenderStats msg from ros thread to GUI thread
	void senderStatsReceived(const nimbro_topic_transport::SenderStatsConstPtr& msg);

private Q_SLOTS:
	void handleSenderStats(const nimbro_topic_transport::SenderStatsConstPtr& msg);
	void updateGroupInfo();
	void updatePlot();
	void clearPlot();
	void handleClickedLegend(QCPLegend *legend, QCPAbstractLegendItem *item, QMouseEvent *event);

private:
	struct GraphValue
	{
		float value; // bandwidth value
		double timestamp; // in seconds since epoch
		double last_timestamp; // in seconds, timestamp of last point
		QCPGraph* graph;
	};

	bool groupFromYaml(const std::string& yaml, GroupMap* map);

	QCustomPlot* m_plot;
	QComboBox* m_connectionBox;
	QTimer m_plotTimer;


	ros::Subscriber m_sub_senderStats;
	std::map<std::string, GraphValue> m_bandwidths;
	double m_maxBandwidth;
	int m_hue;

	std::vector<std::string> m_connections;

	QString m_grpYamlString;
	GroupMap m_groupMap;
};

// Text Dialog to change Group Settings
class GroupDialog : public QDialog
{
Q_OBJECT
public:
	GroupDialog();
	void setText(QString text);
	QString text();
private:
	QTextEdit* m_tEdit;
};

}
#endif
