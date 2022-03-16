// OpenGL plot widget
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "plot_widget.h"

#include <QTimer>

#include "contrib/imgui/imgui.h"
#include "contrib/imgui/backends/imgui_impl_opengl3.h"
#include "contrib/implot/implot.h"

#include <fontconfig/fontconfig.h>

namespace nimbro_topic_transport
{

PlotWidget::PlotWidget(QWidget* parent)
 : QOpenGLWidget(parent)
{
	QTimer* timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, [&](){ update(); });
	timer->start(50);

	auto fmt = QSurfaceFormat::defaultFormat();
	fmt.setSamples(4);
	setFormat(fmt);
}

PlotWidget::~PlotWidget()
{
	if(m_implot)
		ImPlot::DestroyContext(m_implot);

	if(m_imgui)
		ImGui::DestroyContext(m_imgui);
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
		m_font_small = m_io->Fonts->AddFontFromFileTTF(fontFile.c_str(), std::round(fontInfo().pixelSize() * 0.8), NULL, NULL);
	}
	else
	{
		m_font_small = m_io->Fonts->AddFontDefault();
	}
}

void PlotWidget::resizeGL(int w, int h)
{
	m_io->DisplaySize.x = w;
	m_io->DisplaySize.y = h;
}

void PlotWidget::paintGL()
{
	ImGui::SetCurrentContext(m_imgui);
	ImPlot::SetCurrentContext(m_implot);

	ImGui_ImplOpenGL3_NewFrame();
	ImGui::NewFrame();

	ros::Time now = ros::Time::now();
	float time = (now - m_plotTimeBase).toSec();

	float w = m_io->DisplaySize.x;
	float h = m_io->DisplaySize.y;

	ImGui::SetNextWindowPos({0,0});
	ImGui::SetNextWindowSize(m_io->DisplaySize);
	ImGui::Begin("network", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize);

	if(ImPlot::BeginPlot("##Bandwidth", ImGui::GetContentRegionAvail()))
	{
		ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
		ImPlot::SetupAxes("Time", "Bandwidth [MBit/s]", flags, 0);
		ImPlot::SetupAxisLimits(ImAxis_X1, time - PLOT_HISTORY_SECS, time, ImGuiCond_Always);
// 		ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1000); // FIXME

// 		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
// 		ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, IMPLOT_AUTO, ImVec4{1.0f,0.0f,0.0f,1.0f});

		std::vector<float> timeBuf(PLOT_BUFFER_SIZE, 0.0f);

		// First half
		std::size_t firstHalfSize = std::min<int>(m_timeBuffer.size(), PLOT_BUFFER_SIZE - m_timeBuffer.offset());
		auto it = std::copy(
			m_timeBuffer.data() + m_timeBuffer.offset(),
			m_timeBuffer.data() + m_timeBuffer.offset() + firstHalfSize,
			timeBuf.begin()
		);
		// Second half
		std::copy(
			m_timeBuffer.data(),
			m_timeBuffer.data() + m_timeBuffer.size() - firstHalfSize,
			it
		);

		std::vector<float> current(PLOT_BUFFER_SIZE, 0.0f);
		std::vector<float> next(PLOT_BUFFER_SIZE, 0.0f);

		for(auto& pair : m_topics)
		{
			auto& buf = pair.second.bandwidth;

			if(buf.size() == 0)
				continue;

			for(std::size_t i = 0; i < PLOT_BUFFER_SIZE; ++i)
			{
				next[i] = current[i] + buf.data()[(i + buf.offset()) % PLOT_BUFFER_SIZE] / (1000ULL * 1000ULL);
			}

			ImPlot::PlotShaded(
				pair.first.c_str(),
				timeBuf.data(),
				current.data(), next.data(),
				m_timeBuffer.size()
			);

			std::swap(current, next);
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
	for(auto topicMsg : msg->topics)
	{
		auto it = m_topics.find(topicMsg.name);
		if(it == m_topics.end())
		{
			std::tie(it, std::ignore) = m_topics.emplace(topicMsg.name, m_timeBuffer.size());
		}

		it->second.bandwidth.push_back(topicMsg.bandwidth);
	}

	m_timeBuffer.push_back((msg->header.stamp - m_plotTimeBase).toSec());
}

void PlotWidget::clear()
{
	m_topics.clear();
}

}
