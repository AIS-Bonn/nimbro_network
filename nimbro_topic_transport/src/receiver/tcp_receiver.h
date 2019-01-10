// Receive TCP messages
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TT_RECEIVER_TCP_RECEIVER_H
#define TT_RECEIVER_TCP_RECEIVER_H

#include "../tcp_packet.h"
#include "../message.h"

#include <thread>
#include <mutex>

#include <netinet/in.h>

namespace nimbro_topic_transport
{

class TCPReceiver
{
public:
	typedef std::function<void(const Message::Ptr&)> Callback;

	TCPReceiver();
	~TCPReceiver();

	void setCallback(const Callback& cb);
private:
	class ClientHandler
	{
	public:
		ClientHandler(TCPReceiver* receiver, int fd);
		~ClientHandler();
		void start();
		bool isRunning() const
		{ return m_running; }
	private:
		TCPReceiver* m_receiver;
		int m_fd;
		std::thread m_thread;
		bool m_running = false;
	};

	void thread();
	void handleClient(int fd);

	int m_fd;
	std::thread m_thread;
	bool m_shouldExit = false;

	sockaddr_storage m_remoteAddr;
	socklen_t m_remoteAddrLen;

	Callback m_callback;

	std::mutex m_topicMutex;
	std::map<std::string, Topic::Ptr> m_topics;

	std::list<ClientHandler*> m_handlers;
};

}

#endif

