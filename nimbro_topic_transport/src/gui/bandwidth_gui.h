// Bandwidth display for transferred topics
// Author: Sebastian Schüller

#ifndef BANDWIDTH_GUI
#define BANDWIDTH_GUI

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <nimbro_topic_transport/SenderStats.h>
#include <map>

class QCustomPlot;
class QCPGraph;

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

Q_SIGNALS:
	// Bounce SenderStats msg from ros thread to GUI thread
	void senderStatsReceived(const nimbro_topic_transport::SenderStatsConstPtr& msg);

private Q_SLOTS:
	void handleSenderStats(const nimbro_topic_transport::SenderStatsConstPtr& msg);

private:
	struct GraphValue
	{
		float value;
		double timestamp; // in seconds since epoch
		QCPGraph* graph;
	};

	void updatePlot();

	QCustomPlot* m_plot;
	ros::Subscriber m_sub_senderStats;
	std::map<std::string, GraphValue> m_bandwidths;
};

}
#endif
