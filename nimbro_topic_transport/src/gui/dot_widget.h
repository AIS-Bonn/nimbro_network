// Display a .dot graph
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DOT_WIDGET_H
#define DOT_WIDGET_H

#include <QWidget>

namespace nimbro_topic_transport
{

class DotWidget : public QWidget
{
Q_OBJECT
public:
	DotWidget();
	virtual ~DotWidget();

	void updateGraph(const std::string& dot);

	virtual void paintEvent(QPaintEvent *) override;
private:
	QPixmap m_pixmap;
	std::vector<uint8_t> m_pngBuf;
};

}

#endif
