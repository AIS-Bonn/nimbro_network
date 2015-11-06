// Bandwidth display for transferred topics
// Author: Sebastian Schüller

#include "bandwidth_gui.h"
#include "contrib/qcustomplot/qcustomplot.h"

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>

#include <QLayout>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <QMessageBox>

Q_DECLARE_METATYPE(nimbro_topic_transport::SenderStatsConstPtr)

static const int WINDOW_SIZE = 64;
static const int BRUSH_VALUE = 180;
static const int BRUSH_SATURATION = 150;
static const std::string DEFAULT_GROUPS =
	"cameras: [\"/camera_center/h264\", \"/camera_left/h264\", \"/camera_right/h264\"]\n"
	"hand_cameras: [\"/camera_left_hand/h264\", \"/camera_right_hand/h264\"]\n"
	"network_stats: [\"/network/sender_stats\", \"/network/receiver_stats\"]\n";

inline double bpsToKbps(double bps)
{
	return bps/1000.0;
}

inline double bpsToMbps(double bps)
{
	return bpsToKbps(bps)/1000.0;
}

namespace nimbro_topic_transport
{
BandwidthGui::BandwidthGui()
 : m_maxBandwidth(0)
 , m_hue(175)
{}

BandwidthGui::~BandwidthGui()
{}

void BandwidthGui::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_plot = new QCustomPlot();
	m_plot->legend->setVisible(true);
	m_plot->legend->setIconBorderPen(Qt::NoPen);
	m_plot->yAxis->setLabel("Kb/s");


	QWidget* wrapper = new QWidget();
	m_connectionBox = new QComboBox(wrapper);
	QPushButton* groupBtn = new QPushButton("Group Settings", wrapper);
	QGridLayout* gl = new QGridLayout(wrapper);
	gl->addWidget(m_plot, 0, 0, 1, 2);
	gl->addWidget(m_connectionBox, 1, 0);
	gl->addWidget(groupBtn, 1, 1);
	wrapper->setLayout(gl);
	wrapper->setWindowTitle("Bandwidth");
	ctx.addWidget(wrapper);

	connect(
		m_plot, SIGNAL(legendClick(QCPLegend *, QCPAbstractLegendItem *, QMouseEvent *)),
		this, SLOT(handleClickedLegend(QCPLegend*, QCPAbstractLegendItem*, QMouseEvent*))
	);

	connect(
		m_connectionBox, SIGNAL(currentIndexChanged(int)),
		this, SLOT(clearPlot())
	);

	connect(
		groupBtn, SIGNAL(pressed()),
		this, SLOT(updateGroupInfo())
	);

	qRegisterMetaType<nimbro_topic_transport::SenderStatsConstPtr>();

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
	m_plot->xAxis->setAutoTickStep(true);
	m_plot->yAxis->setRangeLower(0);
	QCPLayoutGrid* subLayout = new QCPLayoutGrid();
	QCPLayoutElement* dummy = new QCPLayoutElement();
	m_plot->plotLayout()->addElement(0, 1, subLayout);
	subLayout->addElement(0, 0, m_plot->legend);
	subLayout->addElement(1, 0, dummy);
	subLayout->setRowStretchFactor(0, 0.01);
	m_plot->plotLayout()->setColumnStretchFactor(1, 0.01);

	connect(
		m_plot->xAxis, SIGNAL(rangeChanged(QCPRange)),
		m_plot->xAxis2, SLOT(setRange(QCPRange))
	);
	connect(
		m_plot->yAxis, SIGNAL(rangeChanged(QCPRange)),
		m_plot->yAxis2, SLOT(setRange(QCPRange))
	);

	connect(
		&m_plotTimer, SIGNAL(timeout()),
		this, SLOT(updatePlot())
	);
	m_plotTimer.start(0);

}

void BandwidthGui::shutdownPlugin()
{
	m_sub_senderStats.shutdown();
}

void BandwidthGui::clearPlot()
{
	m_maxBandwidth = 0;
	m_bandwidths.clear();
	m_plot->clearGraphs();
}

void BandwidthGui::handleSenderStats(const nimbro_topic_transport::SenderStatsConstPtr& msg)
{
	// Update connection info
	std::stringstream ss;
	ss << "[" << msg->protocol << "] ";
	ss << msg->host << ":" << msg->source_port << " -> ";
	ss << msg->destination << ":" << msg->destination_port;
	QString connection = QString::fromStdString(ss.str());
	if(m_connectionBox->findText(connection) == -1)
		m_connectionBox->addItem(connection);

	if(connection != m_connectionBox->currentText())
		return;

	for(auto& gv : m_bandwidths)
		gv.second.value = 0.0;

	for(const auto& topic : msg->topics)
	{
		//Look up if topic is in group
		std::string name = topic.name;
		for(const auto& group : m_groupMap)
		{
			const auto& members = group.second;
			if(std::find(members.begin(), members.end(), name) != members.end())
			{
				name = group.first;
				break;
			}
		}

		if(m_bandwidths.find(name) == m_bandwidths.end())
		{
			// Add and configure new graph to plot
			auto graph = m_plot->addGraph();
			graph->setName(QString::fromStdString(name));
			graph->setPen(QPen(QColor::fromHsv(0,110,150)));
			graph->setBrush(QBrush(QColor::fromHsv(m_hue, BRUSH_SATURATION, BRUSH_VALUE)));
			m_hue = (m_hue + 137) % 360;
			m_bandwidths[name].graph = graph;
			m_bandwidths[name].last_timestamp = std::numeric_limits<double>::max();
		}
		m_bandwidths[name].value += bpsToKbps(topic.bandwidth);
		m_bandwidths[name].timestamp = msg->header.stamp.toSec();
	}
}

void BandwidthGui::updateGroupInfo()
{
	QString temp;
	GroupDialog dial; //ToDo find correct parent
	dial.setText(m_grpYamlString);

	int result = dial.exec();
	if (result != QDialog::Accepted)
		return;
	temp = dial.text();
	if (m_grpYamlString == temp)
		return;

	GroupMap groupMap;
	if(!groupFromYaml(temp.toStdString(), &groupMap))
	{
		QMessageBox::warning(m_plot->parentWidget(), "Warning!", "Malformed YAML");
		return;
	}
	m_grpYamlString = temp;
	clearPlot();
	m_groupMap = groupMap;

	return;
}

bool BandwidthGui::groupFromYaml(const std::string& yaml, GroupMap* groupMap)
{
	YAML::Node grpInfo = YAML::Load(yaml);
	if (grpInfo.IsNull())
	{
		return true;
	}
	if (grpInfo.Type() != YAML::NodeType::Map)
	{
		return false;
	}
	for (const auto& pair : grpInfo)
	{
		if(pair.first.Type() != YAML::NodeType::Scalar)
		{
			return false;
		}
		std::string key = pair.first.as<std::string>();
		for (const auto& element : pair.second)
		{
			if(element.Type() != YAML::NodeType::Scalar)
			{
				return false;
			}
			(*groupMap)[key].push_back(element.as<std::string>());
		}
	}
	return true;
}



void BandwidthGui::updatePlot()
{
	double plotValue = 0;

	QCPGraph* prevGraph = nullptr;

	for(auto& bw : m_bandwidths)
	{
		auto& gv = bw.second;
		plotValue += gv.value;

		if(prevGraph)
			gv.graph->setChannelFillGraph(prevGraph);
		prevGraph = gv.graph;

		gv.graph->addData(gv.timestamp, plotValue);
		m_maxBandwidth = std::max(m_maxBandwidth, plotValue);
		m_plot->yAxis->setRangeUpper(m_maxBandwidth + (m_maxBandwidth/50.0));
		gv.graph->removeDataBefore(5*WINDOW_SIZE);
		gv.last_timestamp = gv.timestamp;
	}
	m_plot->xAxis->setRange(ros::Time::now().toSec() + 0.25, WINDOW_SIZE, Qt::AlignRight);
	m_plot->replot();
}

void BandwidthGui::handleClickedLegend(QCPLegend *legend, QCPAbstractLegendItem *item, QMouseEvent *event)
{
	QCPPlottableLegendItem* graphItem = dynamic_cast<QCPPlottableLegendItem*>(item);
	if(!graphItem)
		return;

	auto color = graphItem->plottable()->brush().color();
	auto hue = color.hue() + 180 % 360;
	auto sat = color.saturation();
	auto val = color.value();

	if(sat == BRUSH_SATURATION)
	{
		sat = 255;
		val = 255;
	}
	else
	{
		sat = BRUSH_SATURATION;
		val = BRUSH_VALUE;
	}
	color.setHsv(hue, sat, val);
	graphItem->plottable()->setBrush(QBrush(color));
}

void BandwidthGui::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("connection", m_connectionBox->currentText());
	instanceSettings.setValue("groups", m_grpYamlString);
}

void BandwidthGui::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	if(instanceSettings.contains("connection"))
		m_connectionBox->addItem(instanceSettings.value("connection").toString());
	if(instanceSettings.contains("groups"))
	{
		m_grpYamlString = instanceSettings.value("groups").toString();
		GroupMap groupMap;
		if(m_grpYamlString.isEmpty())
			m_grpYamlString = QString::fromStdString(DEFAULT_GROUPS);

		if(!groupFromYaml(m_grpYamlString.toStdString(), &groupMap))
		{
			m_grpYamlString = "";
			return;
		}
		m_groupMap = groupMap;
	}
}


////// Group Dialog
GroupDialog::GroupDialog()
 : m_tEdit(new QTextEdit(this))
{
	QPushButton* okBtn = new QPushButton("Accept", this);
	QPushButton* cancelBtn = new QPushButton("Cancel", this);
	QGridLayout* gl = new QGridLayout(this);
	gl->addWidget(m_tEdit, 0, 0, 1, 2);
	gl->addWidget(cancelBtn, 1, 0);
	gl->addWidget(okBtn, 1, 1);
	this->setLayout(gl);

	connect(
		okBtn, SIGNAL(pressed()),
		this, SLOT(accept())
	);

	connect(
		cancelBtn, SIGNAL(pressed()),
		this, SLOT(reject())
	);
}

void GroupDialog::setText(QString text)
{
	m_tEdit->setPlainText(text);
}

QString GroupDialog::text()
{
	return m_tEdit->toPlainText();
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_topic_transport::BandwidthGui, rqt_gui_cpp::Plugin)
