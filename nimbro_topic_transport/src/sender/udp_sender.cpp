// Send messages over UDP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"

#include "../udp_packet.h"

#include <netdb.h>

namespace nimbro_topic_transport
{

namespace
{
	inline uint64_t div_round_up(uint64_t a, uint64_t b)
	{
		return (a + b - 1) / b;
	}
}

UDPSender::UDPSender()
{
	ros::NodeHandle nh("~");

	// Get ROS parameters
	std::string dest_host;
	nh.param("destination_addr", dest_host, std::string("localhost"));

	int dest_port;
	nh.param("destination_port", dest_port, 5050);

	std::string dest_port_str = boost::lexical_cast<std::string>(dest_port);

	// Resolve the destination address
	// note: getaddrinfo() also accepts direct IP addresses
	addrinfo *info = 0;
	if(getaddrinfo(dest_host.c_str(), dest_port_str.c_str(), 0, &info) != 0 || !info)
	{
		ROS_FATAL("Could not lookup destination address\n '%s': %s",
			dest_host.c_str(), strerror(errno)
		);
		throw std::runtime_error(strerror(errno));
	}

	m_fd = socket(info->ai_family, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		ROS_FATAL("Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
	{
		ROS_FATAL("Could not enable SO_BROADCAST flag: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
	m_addrLen = info->ai_addrlen;

	int source_port = dest_port;

	// If we have a specified source port, bind to it.
	if(nh.hasParam("source_port"))
	{
		if(!nh.getParam("source_port", source_port))
		{
			ROS_FATAL("Invalid source_port");
			throw std::runtime_error("Invalid source port");
		}

		std::string source_port_str = boost::lexical_cast<std::string>(source_port);

		addrinfo hints;
		memset(&hints, 0, sizeof(hints));

		hints.ai_flags = AI_PASSIVE;
		hints.ai_family = info->ai_family;
		hints.ai_socktype = SOCK_DGRAM;

		addrinfo* localInfo = 0;
		if(getaddrinfo(NULL, source_port_str.c_str(), &hints, &localInfo) != 0 || !localInfo)
		{
			ROS_FATAL("Could not get local address: %s", strerror(errno));
			throw std::runtime_error("Could not get local address");
		}

		if(bind(m_fd, localInfo->ai_addr, localInfo->ai_addrlen) != 0)
		{
			ROS_FATAL("Could not bind to source port: %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		freeaddrinfo(localInfo);
	}

	freeaddrinfo(info);

	// For statistics messages, we need our own hostname.
	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = ros::this_node::getName();
	m_stats.protocol = "UDP";
	m_stats.host = hostnameBuf;
	m_stats.destination = dest_host;
	m_stats.destination_port = dest_port;
	m_stats.source_port = source_port;
	m_stats.fec = m_fec != 0.0;

	nh.param("label", m_stats.label, std::string());

	m_pub_stats = nh.advertise<nimbro_topic_transport::SenderStats>("/network/sender_stats", 1);

	// Start periodic statistics timer.
	m_statsInterval = ros::WallDuration(2.0);
	m_statsTimer = nh.createWallTimer(m_statsInterval,
		boost::bind(&UDPSender::updateStats, this)
	);
	m_statsTimer.start();
}

UDPSender::~UDPSender()
{
}

UDPSender::Callback UDPSender::createCallback(const Topic::ConstPtr& topic)
{
	TopicInfo info;

	auto config = topic->config;
	if(config.hasMember("fec"))
	{
		info.fec = (double)config["fec"];
		if(info.fec < 0)
			throw std::runtime_error("Invalid FEC percentage.");
	}

	return std::bind(&UDPSender::send, this, info, std::placeholders::_1);
}

void UDPSender::send(const TopicInfo& info, const Message::ConstPtr& msg)
{
	// Acquire a unique message ID
	uint16_t messageID;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		messageID = m_msgID++;
	}

	uint64_t dataSize = sizeof(UDPData::Header) + msg->payload.size();
	uint64_t sourceSymbols = div_round_up(dataSize, UDPPacket::MaxDataSize);
	uint64_t repairSymbols = std::ceil(info.fec * sourceSymbols);

	std::vector<std::vector<uint8_t>> packets;
	std::vector<void*> fecSymbols;

	// Write source packets
	{
		uint64_t remaining = dataSize;
		const uint8_t* readPtr = msg->payload.data();

		for(unsigned int packetID = 0; remaining != 0; ++packetID)
		{
			uint64_t take_now = std::min<uint64_t>(UDPPacket::MaxDataSize, remaining);
			Packet packetBuf;
			packetBuf.data.resize(PACKET_SIZE);

			UDPPacket* packet = reinterpret_cast<UDPPacket*>(packetBuf.data.data());

			packet->header.msg_id = messageID;
			packet->header.source_symbols = sourceSymbols;
			packet->header.repair_symbols = repairSymbols;
			packet->header.symbol_id = packetID;

			packetBuf.length = sizeof(UDPPacket::Header) + take_now;

			if(packetID == 0)
			{
				// First packet includes the data header
				UDPData* data = reinterpret_cast<UDPData*>(packet->data);

				data->header.flags = msg->flags;
				data->header.topic_msg_counter = msg->counter;

				strncpy(data->header.topic_name, msg->topic->name.c_str(), sizeof(data->header.topic_name));
				if(data->header.topic_name[sizeof(data->header.topic_name)-1] != 0)
				{
					ROS_ERROR_THROTTLE(0.5,
						"Topic '%s' is too long. Please shorten the name.",
						msg->topic->name.c_str()
					);
					data->header.topic_name[sizeof(data->header.topic_name)-1] = 0;
				}

				strncpy(data->header.topic_type, msg->type.c_str(), sizeof(data->header.topic_type));
				if(data->header.topic_type[sizeof(data->header.topic_type)-1] != 0)
				{
					ROS_ERROR_THROTTLE(0.5,
						"Topic type '%s' is too long. Please shorten the name.",
						msg->topic->name.c_str()
					);
					data->header.topic_type[sizeof(data->header.topic_type)-1] = 0;
				}

				for(int i = 0; i < 4; ++i)
				{
					std::string md5_part = msg->md5.substr(8*i, 8);
					uint32_t md5_num = strtol(md5_part.c_str(), 0, 16);
					data->header.topic_md5[i] = md5_num;
				}

				data->header.size = msg->payload.size();

				// Fill remainder with data
				take_now -= sizeof(UDPData::Header);
				memcpy(data->data, readPtr, take_now);
				readPtr += take_now;
			}
			else
			{
				// Just data
				memcpy(packet->data, readPtr, take_now);
				readPtr += take_now;
			}

			// Fill any leftover regions (padding for FEC) with zeroes
			memset(packetBuf.data.data() + packetBuf.length, 0,
				packetBuf.data.size() - packetBuf.length
			);
		}
	}

	// Create repair symbols
	{

	}
}

}
