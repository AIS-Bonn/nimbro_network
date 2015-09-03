// Service Display RQT Plugin
// Author: Sebastian Schüller

#ifndef SERVICE_WIDGET
#define SERVICE_WIDGET

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <nimbro_service_transport/ServiceStatus.h>

#include <deque>

#include <QAbstractTableModel>

class QComboBox;
class QTableView;

namespace nimbro_service_transport
{

class ServiceStatusModel : public QAbstractTableModel
{
Q_OBJECT
public:
	explicit ServiceStatusModel(QWidget* parent = 0);
	virtual ~ServiceStatusModel();

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	QVariant data(const QModelIndex& parent = QModelIndex(), int role = Qt::DisplayRole) const override;

	void addService(std::string name, uint8_t status, uint32_t call_id);

public Q_SLOTS:
	void clearData();

private:
	enum columns
	{
		NAME_COL = 0,
		STATUS_COL,
		NUM_COLUMNS
	};

	struct ServiceData
	{
		uint32_t call_id;
		std::string name;
		uint8_t status;
	};
	std::deque<ServiceData> m_data;

};

class ServiceWidget : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	ServiceWidget();
	virtual ~ServiceWidget();

	virtual void initPlugin(qt_gui_cpp::PluginContext & ) override;
	virtual void shutdownPlugin() override;
void saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const override;
void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;

Q_SIGNALS:
	void serviceStatusRecieved(const nimbro_service_transport::ServiceStatusConstPtr& msg);

private Q_SLOTS:
	void handleServiceStatus(const nimbro_service_transport::ServiceStatusConstPtr& msg);

private:


	ros::Subscriber m_sub_serviceStatus;

	ServiceStatusModel* m_model;
	QTableView* m_view;
	QComboBox* m_connectionBox;
};

}

#endif
