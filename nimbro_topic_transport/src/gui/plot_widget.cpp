// OpenGL plot widget
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "plot_widget.h"

#include <QTimer>
#include <QMouseEvent>

#include "contrib/imgui/imgui.h"
#include "contrib/imgui/backends/imgui_impl_opengl3.h"
#include "contrib/implot/implot.h"

#include <fontconfig/fontconfig.h>

namespace nimbro_topic_transport
{

namespace
{
	ImGuiMouseButton imGuiButton(Qt::MouseButton btn)
	{
		switch(btn)
		{
			case Qt::LeftButton: return ImGuiMouseButton_Left;
			case Qt::MiddleButton: return ImGuiMouseButton_Middle;
			case Qt::RightButton: return ImGuiMouseButton_Right;
			default: return -1;
		}
	}
}

PlotWidget::PlotWidget(QWidget* parent)
 : QOpenGLWidget(parent)
{
	m_updateTimer = new QTimer(this);
	connect(m_updateTimer, &QTimer::timeout, this, [&](){ update(); });
	m_updateTimer->start(50);

	auto fmt = QSurfaceFormat::defaultFormat();
	fmt.setSamples(4);
	setFormat(fmt);

	setMouseTracking(true);
}

PlotWidget::~PlotWidget()
{
	if(m_implot)
		ImPlot::DestroyContext(m_implot);

	if(m_imgui)
		ImGui::DestroyContext(m_imgui);
}

void PlotWidget::mouseMoveEvent(QMouseEvent* event)
{
	if(!m_imgui)
		return;

	ImGui::SetCurrentContext(m_imgui);
	m_io->AddMousePosEvent(event->x(), event->y());
}

void PlotWidget::mousePressEvent(QMouseEvent* event)
{
	if(!m_imgui)
		return;

	int button = imGuiButton(event->button());
	if(button < 0)
		return;

	ImGui::SetCurrentContext(m_imgui);
	m_io->AddMouseButtonEvent(button, true);
}

void PlotWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if(!m_imgui)
		return;

	int button = imGuiButton(event->button());
	if(button < 0)
		return;

	ImGui::SetCurrentContext(m_imgui);
	m_io->AddMouseButtonEvent(button, false);
}

void PlotWidget::initializeGL()
{
	IMGUI_CHECKVERSION();
	m_imgui = ImGui::CreateContext();
	ImGui::SetCurrentContext(m_imgui);

	m_io = &ImGui::GetIO();
	m_io->IniFilename = nullptr;
	ImGui::StyleColorsLight(&ImGui::GetStyle());
	ImGui::GetStyle().Colors[ImGuiCol_WindowBg] = {1.0f, 1.0f, 1.0f, 1.0f};

	m_implot = ImPlot::CreateContext();
	ImPlot::SetCurrentContext(m_implot);

	ImGui_ImplOpenGL3_Init("#version 130");

	std::string fontFile;
	{
		FcConfig* config = FcInitLoadConfigAndFonts();

		// configure the search pattern,
		// assume "name" is a std::string with the desired font name in it
		auto fontName = fontInfo().family().toLocal8Bit();
		FcPattern* pat = FcNameParse((const FcChar8*)(fontName.data()));
		FcConfigSubstitute(config, pat, FcMatchPattern);
		FcDefaultSubstitute(pat);

		// find the font
		FcResult res;
		FcPattern* font = FcFontMatch(config, pat, &res);
		if (font)
		{
			FcChar8* file = NULL;
			if (FcPatternGetString(font, FC_FILE, 0, &file) == FcResultMatch)
			{
				// save the file to another std::string
				fontFile = (char*)file;
			}
			FcPatternDestroy(font);
		}

		FcPatternDestroy(pat);
	}

	if(!fontFile.empty())
	{
		m_io->Fonts->AddFontFromFileTTF(fontFile.c_str(), fontInfo().pixelSize(), NULL, NULL);
	}
	else
	{
	}
}

void PlotWidget::resizeGL(int w, int h)
{
	m_io->DisplaySize.x = w;
	m_io->DisplaySize.y = h;

	// Adapt update rate for smooth animation
	float pixelSpeed = static_cast<float>(w) / ScrollingBuffer::HISTORY_SECS;
	constexpr float MAX_PIXEL_PER_UPDATE = 2.0f;
	float updateRate = pixelSpeed / MAX_PIXEL_PER_UPDATE;

	updateRate = std::max(5.0f, std::min(60.0f, updateRate));

	m_updateTimer->setInterval(1.0f / updateRate * 1000);
}

void PlotWidget::paintGL()
{
	ImGui::SetCurrentContext(m_imgui);
	ImPlot::SetCurrentContext(m_implot);

	ImGui_ImplOpenGL3_NewFrame();
	ImGui::NewFrame();

	ros::Time now = ros::Time::now();
	float time = (now - m_plotTimeBase).toSec();

	ImGui::SetNextWindowPos({0,0});
	ImGui::SetNextWindowSize(m_io->DisplaySize);
	ImGui::Begin("network", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize);

	if(ImPlot::BeginPlot("##Bandwidth", ImGui::GetContentRegionAvail()))
	{
		ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
		ImPlot::SetupAxes("Time", "Bandwidth [MBit/s]", flags, 0);

		float timeOffset = 1.0f / ScrollingBuffer::DATA_RATE_HZ;
		ImPlot::SetupAxisLimits(ImAxis_X1,
			time - timeOffset - ScrollingBuffer::HISTORY_SECS,
			time - timeOffset,
			ImGuiCond_Always
		);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0, std::max(1.1f * m_buffer.maximum(), 1.0f), ImGuiCond_Always);

		for(int row = m_topics.size()-1; row >= 0; --row)
		{
			if(row == 0)
			{
				ImPlot::PlotShaded(
					m_topics[row].c_str(),
					m_buffer.timeData(),
					m_buffer.rowAccData(0),
					m_buffer.size(),
					0.0f,
					m_buffer.offset()
				);
			}
			else
			{
				ImPlot::PlotShaded(
					m_topics[row].c_str(),
					m_buffer.timeData(),
					m_buffer.rowAccData(row-1),
					m_buffer.rowAccData(row),
					m_buffer.size(),
					m_buffer.offset()
				);
			}
		}

		ImPlot::EndPlot();
	}

	ImGui::End();

	ImGui::Render();
	glViewport(0, 0, width(), height());
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void PlotWidget::integrateData(const nimbro_topic_transport::SenderStatsConstPtr& msg)
{
	std::vector<float> data(m_buffer.rows(), 0.0f);

	for(auto topicMsg : msg->topics)
	{
		auto it = std::lower_bound(m_topics.begin(), m_topics.end(), topicMsg.name);
		int idx = it - m_topics.begin();

		float mbits = topicMsg.bandwidth / 1000ULL / 1000ULL; // MBit/s

		if(it == m_topics.end() || *it != topicMsg.name)
		{
			m_buffer.addRow(idx);
			data.insert(data.begin() + idx, mbits);
			m_topics.insert(it, topicMsg.name.c_str());
		}
		else
			data[idx] = mbits;
	}

	m_buffer.push_back((msg->header.stamp - m_plotTimeBase).toSec(), data.data());
}

void PlotWidget::clear()
{
	m_topics.clear();
	m_buffer.reset(0);
}

}
