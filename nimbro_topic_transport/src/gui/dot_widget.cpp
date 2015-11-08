// Display a .dot graph
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "dot_widget.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>
#include <sys/wait.h>


#include <QPainter>

static ssize_t runDot(const std::string& dot, int width, int height, uint8_t* pngOut, size_t pngOutSize)
{
	int inputPipe[2];
	int outputPipe[2];

	if(pipe(inputPipe) != 0 || pipe(outputPipe) != 0)
		throw std::runtime_error("Could not create pipe");

	int pid = fork();
	if(pid == 0)
	{
		// Child
		close(inputPipe[1]);
		close(outputPipe[0]);

		dup2(inputPipe[0], STDIN_FILENO);
		dup2(outputPipe[1], STDOUT_FILENO);

		char sizeBuf[256];
		int dpi = 400;
		snprintf(sizeBuf, sizeof(sizeBuf), "-Gsize=%f,%f", ((float)width)/dpi, ((float)height)/dpi);

		if(execlp("dot", "dot", "-Tpng", sizeBuf, "-Gdpi=400", "/dev/stdin", (char*)NULL) != 0)
		{
			perror("execlp() failed");
			throw std::runtime_error("Could not execute dot");
			exit(1);
		}

		// Cannot reach this
		return -1;
	}
	else
	{
		// Parent
		close(inputPipe[0]);
		close(outputPipe[1]);

		size_t written = 0;
		size_t len = dot.length();
		while(written != len)
		{
			int ret = write(inputPipe[1], dot.c_str() + written, len - written);
			if(ret <= 0)
				perror("Could not write");

			written += ret;
		}
		close(inputPipe[1]);

		size_t readBytes = 0;
		while(1)
		{
			if(pngOutSize == readBytes)
			{
				throw std::runtime_error("png buffer is to small");
			}

			int ret = read(outputPipe[0], pngOut + readBytes, pngOutSize - readBytes);
			if(ret == 0)
				break;
			else if(ret < 0)
			{
				perror("Could not read");
				throw std::runtime_error("Could not read");
			}

			readBytes += ret;
		}
		close(outputPipe[0]);

		int status;
		waitpid(pid, &status, 0);
		if(status != 0)
			fprintf(stderr, "dot exited with error status %d\n", status);

		return readBytes;
	}
}

namespace nimbro_topic_transport
{

DotWidget::DotWidget()
 : m_pngBuf(1024 * 1024)
{
}

DotWidget::~DotWidget()
{
}

void DotWidget::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.fillRect(rect(), Qt::white);

	painter.setRenderHint(QPainter::SmoothPixmapTransform);

	float scale_x = ((float)width()) / m_pixmap.width();
	float scale_y = ((float)height()) / m_pixmap.height();

	if(scale_x > scale_y)
	{
		int newWidth = scale_y * m_pixmap.width();
		QRect pixmapRect((width() - newWidth)/2, 0, newWidth, height());
		painter.drawPixmap(pixmapRect, m_pixmap);
	}
	else
	{
		int newHeight = scale_x * m_pixmap.height();
		QRect pixmapRect(0, (height() - newHeight)/2, width(), newHeight);
		painter.drawPixmap(pixmapRect, m_pixmap);
	}
}

void DotWidget::updateGraph(const std::string& dot)
{
	ssize_t pngSize = runDot(dot, width(), height(), m_pngBuf.data(), m_pngBuf.size());

	m_pixmap.loadFromData(m_pngBuf.data(), pngSize, "png");

	update();
}

}
