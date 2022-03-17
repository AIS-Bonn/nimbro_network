// Protocol for standalone executable
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef STANDALONE_PROTO_H
#define STANDALONE_PROTO_H

#include <string>
#include <vector>

namespace proto
{
	struct WifiStats
	{
		bool is_wifi_device = 0;
		bool associated = 0;
		std::uint32_t associated_since = 0;
		std::string ssid;
		std::uint16_t frequency_mhz = 0;

		std::int16_t signal_dbm = 0;
		std::int16_t signal_avg_dbm = 0;
		std::int16_t beacon_signal_dbm = 0;

		std::uint8_t tx_mcs = 0;
		std::uint16_t tx_bw = 0;
		std::uint64_t tx_bitrate = 0;

		std::uint8_t rx_mcs = 0;
		std::uint16_t rx_bw = 0;
		std::uint64_t rx_bitrate = 0;

		template <class T>
		void Serialize(T& archive)
		{
			archive
				& is_wifi_device
				& associated
				& associated_since
				& ssid
				& frequency_mhz
				& signal_dbm
				& signal_avg_dbm
				& beacon_signal_dbm
				& tx_mcs
				& tx_bw
				& tx_bitrate
				& rx_mcs
				& rx_bw
				& rx_bitrate
			;
		}
	};

	struct Interface
	{
		std::string interface_name;
		std::uint64_t max_bits_per_second = 0;
		bool duplex = false;
		double tx_bandwidth = 0;
		double rx_bandwidth = 0;
		double tx_packet_rate = 0;
		double rx_packet_rate = 0;
		WifiStats wifi;

		template <class T>
		void Serialize(T& archive)
		{
			archive
				& interface_name
				& max_bits_per_second
				& duplex
				& tx_bandwidth
				& rx_bandwidth
				& tx_packet_rate
				& rx_packet_rate
				& wifi
			;
		}
	};

	struct NetworkStats
	{
		std::uint64_t stamp_secs = 0;
		std::uint64_t stamp_nsecs = 0;

		std::string host;
		std::vector<Interface> interfaces;

		template <class T>
		void Serialize(T& archive)
		{
			archive
				& stamp_secs
				& stamp_nsecs
				& host
				& interfaces
			;
		}
	};
}

#endif
