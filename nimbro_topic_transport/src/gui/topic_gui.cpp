// Diagnostic GUI for nimbro_topic_transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_gui.h"

#include <QMetaType>

#include <ros/node_handle.h>

#include <set>

#include <pluginlib/class_list_macros.h>

#include <sstream>

Q_DECLARE_METATYPE(nimbro_topic_transport::SenderStatsConstPtr)
Q_DECLARE_METATYPE(nimbro_topic_transport::ReceiverStatsConstPtr)

namespace nimbro_topic_transport
{

TopicGUI::TopicGUI()
{
}

TopicGUI::~TopicGUI()
{
}

void TopicGUI::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_w = new DotWidget;
	ctx.addWidget(m_w);

	qRegisterMetaType<nimbro_topic_transport::SenderStatsConstPtr>();
	qRegisterMetaType<nimbro_topic_transport::ReceiverStatsConstPtr>();

	connect(
		this, SIGNAL(senderStatsReceived(nimbro_topic_transport::SenderStatsConstPtr)),
		this, SLOT(handleSenderStats(nimbro_topic_transport::SenderStatsConstPtr)),
		Qt::QueuedConnection
	);

	m_sub_senderStats = getPrivateNodeHandle().subscribe(
		"/network/sender_stats", 1, &TopicGUI::senderStatsReceived, this
	);

	connect(
		this, SIGNAL(receiverStatsReceived(nimbro_topic_transport::ReceiverStatsConstPtr)),
		this, SLOT(handleReceiverStats(nimbro_topic_transport::ReceiverStatsConstPtr)),
		Qt::QueuedConnection
	);

	m_sub_receiverStats = getPrivateNodeHandle().subscribe(
		"/network/receiver_stats", 1, &TopicGUI::receiverStatsReceived, this
	);

	QTimer* updateTimer = new QTimer(this);
	connect(updateTimer, SIGNAL(timeout()), SLOT(update()));
	updateTimer->start(2000);
}

void TopicGUI::shutdownPlugin()
{
	m_sub_senderStats.shutdown();
	m_sub_receiverStats.shutdown();
}

void TopicGUI::handleSenderStats(const SenderStatsConstPtr& msg)
{
	ConnectionIdentifier ident;
	ident.dest = msg->destination;
	ident.protocol = msg->protocol;
	ident.sourcePort = msg->source_port;
	ident.destPort = msg->destination_port;

	m_senderStats[ident] = msg;

	update();
}

void TopicGUI::handleReceiverStats(const ReceiverStatsConstPtr& msg)
{
	ConnectionIdentifier ident;
	ident.dest = msg->host;
	ident.protocol = msg->protocol;
	ident.sourcePort = msg->remote_port;
	ident.destPort = msg->local_port;

	m_receiverStats[ident] = msg;

	update();
}

static std::string sanitize(std::string arg)
{
	for(size_t i = 0; i < arg.length(); ++i)
	{
		if(!isalnum(arg[i]))
			arg[i] = '_';
	}

	return arg;
}

void TopicGUI::update()
{
	ros::Time now = ros::Time::now();
	ros::Duration timeoutDuration(4.0);

	std::stringstream ss;
	ss << "digraph G {\n";
	ss << "K=0.6;\n";

	std::set<ConnectionIdentifier> connections;
	std::set<std::string> hosts;

	for(auto pair : m_receiverStats)
	{
		if(now - pair.second->header.stamp > timeoutDuration)
			continue;

		connections.insert(pair.first);
		hosts.insert(pair.second->remote);
		hosts.insert(pair.first.dest);
	}
	for(auto pair : m_senderStats)
	{
		if(now - pair.second->header.stamp > timeoutDuration)
			continue;

		connections.insert(pair.first);
		hosts.insert(pair.second->host);
		hosts.insert(pair.first.dest);
	}

	for(auto& host : hosts)
	{
		ss << "node [ label = \"" << host << "\" ]; node_" << sanitize(host) << ";\n";
	}

	for(const auto& connection : connections)
	{
		auto sender_it = m_senderStats.find(connection);
		auto receiver_it = m_receiverStats.find(connection);

		SenderStatsConstPtr sender;
		if(sender_it != m_senderStats.end())
			sender = sender_it->second;

		ReceiverStatsConstPtr receiver;
		if(receiver_it != m_receiverStats.end())
			receiver = receiver_it->second;

		std::string source = sender ? sender->host : receiver->remote;

		ss << "node_" << sanitize(source) << " -> node_" << sanitize(connection.dest) << "[ ";

		ss << "label=\"";

		std::string label;
		if(sender_it != m_senderStats.end() && !sender_it->second->label.empty())
			label = sender_it->second->label;
		else if(receiver_it != m_receiverStats.end() && !receiver_it->second->label.empty())
			label = receiver_it->second->label;

		if(!label.empty())
			ss << label << "\n";

		ss << connection.protocol << " " << connection.sourcePort << " -> " << connection.destPort << "\n";

		float bandwidth = 0.0;
		int div = 0;
		if(sender_it != m_senderStats.end() && now - sender_it->second->header.stamp < timeoutDuration)
		{
			bandwidth += sender_it->second->bandwidth;
			div++;
		}
		if(receiver_it != m_receiverStats.end() && now - receiver_it->second->header.stamp < timeoutDuration)
		{
			bandwidth += receiver_it->second->bandwidth;
			div++;
		}

		char bwBuf[100];
		snprintf(bwBuf, sizeof(bwBuf), "%.2f MBit/s", bandwidth / 1024 / 1024);

		ss << bwBuf;
		ss << "\"";

		ss << " ];\n";
	}

	ss << "}";

	m_w->updateGraph(ss.str());
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_topic_transport::TopicGUI, rqt_gui_cpp::Plugin)
