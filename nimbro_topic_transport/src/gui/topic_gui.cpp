// Display for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "topic_gui.h"

#include <pluginlib/class_list_macros.hpp>
#include <ros/node_handle.h>
#include <rosfmt/rosfmt.h>

#include <QOpenGLWidget>
#include <QTimer>
#include <QPushButton>

#include <random>

#include <ros/master.h>


Q_DECLARE_METATYPE(nimbro_topic_transport::SenderStatsConstPtr)

namespace nimbro_topic_transport
{

TopicGUI::TopicGUI()
{
	qRegisterMetaType<nimbro_topic_transport::SenderStatsConstPtr>();
}

void TopicGUI::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	QWidget* w = new QWidget;
	m_ui.setupUi(w);
	ctx.addWidget(w);

	connect(this, &TopicGUI::dataReceived, this, [=](const nimbro_topic_transport::SenderStatsConstPtr& msg){
		m_ui.plotWidget->integrateData(msg);
	}, Qt::QueuedConnection);

	connect(m_ui.refreshTopicsButton, &QPushButton::clicked, this, &TopicGUI::refreshTopics);

	connect(m_ui.topicBox, &QComboBox::currentTextChanged, this, &TopicGUI::handleTopicChange);
}

void TopicGUI::shutdownPlugin()
{
	m_sub.shutdown();
}

void TopicGUI::refreshTopics()
{
	QString selected = m_ui.topicBox->currentText();

	std::vector<ros::master::TopicInfo> topics;
	ros::master::getTopics(topics);

	m_ui.topicBox->clear();

	for(auto& topic : topics)
	{
		if(topic.datatype == "nimbro_topic_transport/SenderStats")
			m_ui.topicBox->addItem(QString::fromStdString(topic.name));
	}

	m_ui.topicBox->addItem("");

	// restore previous selection
	selectTopic(selected);
}

void TopicGUI::selectTopic(const QString& topic)
{
	int index = m_ui.topicBox->findText(topic);
	if(index == -1)
	{
		// add topic name to list if not yet in
		m_ui.topicBox->addItem(topic);
		index = m_ui.topicBox->findText(topic);
	}
	m_ui.topicBox->setCurrentIndex(index);
}

void TopicGUI::handleTopicChange()
{
	m_ui.plotWidget->clear();
	m_sub = ros::Subscriber();

	QString topic = m_ui.topicBox->currentText();
	if(topic.isEmpty())
		return;

	m_sub = getPrivateNodeHandle().subscribe(
		topic.toStdString(), 10,
		&TopicGUI::dataReceived, this
	);
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_topic_transport::TopicGUI, rqt_gui_cpp::Plugin)
