// RTNetlink interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_NET_MONITOR_RTNETLINK_H
#define NIMBRO_NET_MONITOR_RTNETLINK_H

#include <memory>
#include <optional>
#include <map>

class RTNetlink
{
public:
	class NLException : public std::runtime_error
	{
	public:
		explicit NLException(const std::string& str) : std::runtime_error{str}
		{}
	};

	class RetryException : public NLException
	{
	public:
		explicit RetryException(const std::string& str) : NLException{str}
		{}
	};

	struct Stats
	{
		std::uint64_t rx_bytes = 0;
		std::uint64_t rx_packets = 0;
		std::uint64_t tx_bytes = 0;
		std::uint64_t tx_packets = 0;
	};

	RTNetlink();
	~RTNetlink();

	RTNetlink(const RTNetlink&) = delete;
	RTNetlink& operator=(const RTNetlink&) = delete;

	std::optional<int> getLinkByName(const std::string& name);
	std::map<std::string, Stats> getLinkStats();

private:
	class Private;
	std::unique_ptr<Private> m_d;
};

#endif
