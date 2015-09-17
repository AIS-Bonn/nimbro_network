// Service Display RQT Plugin
// Author: Sebastian Schüller

#include "service_widget.h"

#include <QComboBox>
#include <QTableView>
#include <QVBoxLayout>
#include <QHeaderView>

#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>

Q_DECLARE_METATYPE(nimbro_service_transport::ServiceStatusConstPtr)

static const QColor CLR_SUCCESS  = QColor::fromHsv(120, 100, 200);
static const QColor CLR_FAILURE  = QColor::fromHsv(0,   100, 200);
static const QColor CLR_TIMEOUT  = QColor::fromHsv(0,   60,  180);
static const QColor CLR_PROGRESS = QColor::fromHsv(0,   0,   200);

namespace nimbro_service_transport
{

ServiceWidget::ServiceWidget()
 : rqt_gui_cpp::Plugin()
{
}

ServiceWidget::~ServiceWidget()
{}

void ServiceWidget::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	QWidget* wrapper = new QWidget();
	m_view = new QTableView(wrapper);
	m_model = new ServiceStatusModel(wrapper);
	m_connectionBox = new QComboBox(wrapper);
	QVBoxLayout* vbl = new QVBoxLayout(wrapper);

	m_view->setModel(m_model);

	qRegisterMetaType<nimbro_service_transport::ServiceStatusConstPtr>();

	connect(
		m_connectionBox, SIGNAL(currentIndexChanged(int)),
		m_model, SLOT(clearData())
	);

	vbl->addWidget(m_view);
	vbl->addWidget(m_connectionBox);
	wrapper->setLayout(vbl);
	ctx.addWidget(wrapper);

	m_sub_serviceStatus = getPrivateNodeHandle().subscribe(
		"/network/service_status", 1, &ServiceWidget::serviceStatusRecieved, this
	);

	connect(
		this, SIGNAL(serviceStatusRecieved(const nimbro_service_transport::ServiceStatusConstPtr&)),
		this, SLOT(handleServiceStatus(const nimbro_service_transport::ServiceStatusConstPtr&))
	);

	connect(
		m_model, SIGNAL(rowsInserted(const QModelIndex&, int, int)),
		m_view, SLOT(scrollToBottom())
	);
}

void ServiceWidget::shutdownPlugin()
{
	m_sub_serviceStatus.shutdown();
}

void ServiceWidget::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("tableViewState", m_view->horizontalHeader()->saveState());
	instanceSettings.setValue("connection", m_connectionBox->currentText());
}

void ServiceWidget::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	if(instanceSettings.contains("tableViewState"))
		m_view->horizontalHeader()->restoreState(instanceSettings.value("tableViewState").toByteArray());
	if(instanceSettings.contains("connection"))
		m_connectionBox->addItem(instanceSettings.value("connection").toString());
}

void ServiceWidget::handleServiceStatus(const nimbro_service_transport::ServiceStatusConstPtr& msg)
{
	std::stringstream ss;
	ss << msg->host << " -> ";
	ss << msg->remote << ":" << msg->remote_port;
	QString connection = QString::fromStdString(ss.str());
	if(m_connectionBox->findText(connection) == -1)
		m_connectionBox->addItem(connection);

	if(connection != m_connectionBox->currentText())
		return;

	m_model->addService(msg->service, msg->status, msg->call_id);
}


///// ServiceStatusModel

ServiceStatusModel::ServiceStatusModel(QWidget* parent)
 : QAbstractTableModel(parent)
{}

ServiceStatusModel::~ServiceStatusModel()
{}

int ServiceStatusModel::rowCount(const QModelIndex& parent) const
{
	return (int)m_data.size();
}

int ServiceStatusModel::columnCount(const QModelIndex& parent) const
{
	return 2;
}

QVariant ServiceStatusModel::data(const QModelIndex& parent, int role) const
{
	switch(role)
	{
		case Qt::DisplayRole:
			if(parent.column() == NAME_COL)
				return QString::fromStdString(m_data[parent.row()].name);
			if(parent.column() == STATUS_COL)
			{
				switch(m_data[parent.row()].status)
				{
					case nimbro_service_transport::ServiceStatus::STATUS_FINISHED_SUCCESS:
						return "Success (finished)";
					case nimbro_service_transport::ServiceStatus::STATUS_FINISHED_ERROR:
						return "Error (finished)";
					case nimbro_service_transport::ServiceStatus::STATUS_IN_PROGRESS:
						return "In Progress";
					case nimbro_service_transport::ServiceStatus::STATUS_TIMEOUT:
						return "Timeout";
					case nimbro_service_transport::ServiceStatus::STATUS_CONNECTION_ERROR:
						return "Connection Error";
					default:
						return "Unknown";
				}
			}

		case Qt::BackgroundRole:
			switch(m_data[parent.row()].status)
			{
				case nimbro_service_transport::ServiceStatus::STATUS_FINISHED_SUCCESS:
					return QBrush(CLR_SUCCESS);
				case nimbro_service_transport::ServiceStatus::STATUS_IN_PROGRESS:
					return QBrush(CLR_PROGRESS);
				case nimbro_service_transport::ServiceStatus::STATUS_TIMEOUT:
					return QBrush(CLR_TIMEOUT);
				case nimbro_service_transport::ServiceStatus::STATUS_FINISHED_ERROR:
				case nimbro_service_transport::ServiceStatus::STATUS_CONNECTION_ERROR:
					return QBrush(CLR_FAILURE);
				default:
					return QVariant();
			}

		default:
			return QVariant();
	}
}

void ServiceStatusModel::addService(std::string name, uint8_t status, uint32_t call_id)
{
	for(auto& sd : m_data)
	{
		if(call_id == sd.call_id)
		{
			sd.status = status;
			dataChanged(QModelIndex(), QModelIndex());
			return;
		}
	}

	ServiceData data;
	data.name = name;
	data.status = status;
	data.call_id = call_id;

	if(m_data.size() == 100)
	{
		beginRemoveRows(QModelIndex(), 0, 0);
		m_data.pop_front();
		endRemoveRows();
	}

	beginInsertRows(QModelIndex(), rowCount(), rowCount());
	m_data.push_back(data);
	endInsertRows();
}

void ServiceStatusModel::clearData()
{
	beginResetModel();
	m_data.clear();
	endResetModel();
}



};

PLUGINLIB_EXPORT_CLASS(nimbro_service_transport::ServiceWidget, rqt_gui_cpp::Plugin)
