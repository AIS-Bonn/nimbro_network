// Obtain IP routes from the kernel
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "route.h"

#include <libmnl/libmnl.h>

#include <netdb.h>

#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#include <net/if.h>

#include <cerrno>
#include <cstring>
#include <functional>
#include <sstream>

#include <fmt/format.h>

namespace route
{

Cache::Cache()
{
	auto deleter = std::bind(mnl_socket_close, std::placeholders::_1);
	m_mnl.reset(mnl_socket_open(NETLINK_ROUTE), deleter);

	if(!m_mnl)
	{
		std::stringstream ss;
		ss << "Could not open Netlink socket for routing: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	if(mnl_socket_bind(m_mnl.get(), 0, MNL_SOCKET_AUTOPID) < 0)
	{
		std::stringstream ss;
		ss << "Could not bind Netlink socket for routing: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

Cache::~Cache()
{
}

namespace
{
	struct CallbackHelper
	{
		using MsgCb = std::function<int(const nlmsghdr*)>;
		using AttrCb = std::function<int(const nlattr*)>;

		CallbackHelper(const MsgCb& cb, const AttrCb& attrCb)
		 : m_cb(cb)
		 , m_attrCb(attrCb)
		{}

		static int callback(const nlmsghdr* nlh, void* arg)
		{
			CallbackHelper* helper = reinterpret_cast<CallbackHelper*>(arg);
			return helper->m_cb(nlh);
		}

		static int attrCallback(const nlattr* attr, void* arg)
		{
			CallbackHelper* helper = reinterpret_cast<CallbackHelper*>(arg);
			return helper->m_attrCb(attr);
		}

		MsgCb m_cb;
		AttrCb m_attrCb;
	};
}

std::string Cache::obtainInterfaceForHost(const std::string& host)
{
	// Cache lookup
	{
		auto it = m_interfaceCache.find(host);
		if(it != m_interfaceCache.end())
			return it->second;
	}

	int ai_family;
	std::vector<uint8_t> addrbuf;

	// DNS lookup
	{
		addrinfo* addr = nullptr;
		if(getaddrinfo(host.c_str(), nullptr, nullptr, &addr) != 0 || !addr)
		{
			fmt::print(stderr, "Could not resolve host '{}'\n", host.c_str());
			return {};
		}

		std::shared_ptr<addrinfo> addrPtr(addr, freeaddrinfo);

		ai_family = addr->ai_family;

		switch(ai_family)
		{
			case AF_INET:
			{
				const auto* saddr = reinterpret_cast<sockaddr_in*>(addr->ai_addr);
				addrbuf.resize(sizeof(saddr->sin_addr));
				memcpy(addrbuf.data(), &saddr->sin_addr, addrbuf.size());
				break;
			}
			case AF_INET6:
			{
				const auto* saddr = reinterpret_cast<sockaddr_in6*>(addr->ai_addr);
				addrbuf.resize(sizeof(saddr->sin6_addr));
				memcpy(addrbuf.data(), &saddr->sin6_addr, addrbuf.size());
				break;
			}
			default:
			{
				fmt::print(stderr, "Got unknown address type for host '{}'\n", host.c_str());
				return {};
			}
		}
	}

	char buf[MNL_SOCKET_BUFFER_SIZE];
	nlmsghdr *nlh;
	unsigned int seq = time(NULL);

	nlh = mnl_nlmsg_put_header(buf);
	nlh->nlmsg_type = RTM_GETROUTE;
	nlh->nlmsg_flags = NLM_F_REQUEST;
	nlh->nlmsg_seq = seq;

	rtmsg* rtm = reinterpret_cast<rtmsg*>(mnl_nlmsg_put_extra_header(nlh, sizeof(rtmsg)));
	rtm->rtm_family = ai_family;
	rtm->rtm_dst_len = addrbuf.size() * 8;
	rtm->rtm_src_len = 0;
	rtm->rtm_tos = 0;
	rtm->rtm_protocol = RTPROT_UNSPEC;
	rtm->rtm_table = RT_TABLE_UNSPEC;
	rtm->rtm_type = RTN_UNSPEC;
	rtm->rtm_scope = RT_SCOPE_UNIVERSE;
	rtm->rtm_flags = RTM_F_LOOKUP_TABLE;

	mnl_attr_put(nlh, RTA_DST, addrbuf.size(), addrbuf.data());

	unsigned int portid = mnl_socket_get_portid(m_mnl.get());

	if(mnl_socket_sendto(m_mnl.get(), nlh, nlh->nlmsg_len) < 0)
	{
		std::stringstream ss;
		ss << "Could not send netlink message: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}

	int outputInterface = -1;

	CallbackHelper helper([&](const nlmsghdr* nlh){
		// Called for each message
		rtmsg* rm = reinterpret_cast<rtmsg*>(mnl_nlmsg_get_payload(nlh));
		if(!rm)
			return MNL_CB_ERROR;

		mnl_attr_parse(nlh, sizeof(*rm), &CallbackHelper::attrCallback, &helper);

		return MNL_CB_OK;
	}, [&](const nlattr* attr){
		// Called for each attribute
		int type = mnl_attr_get_type(attr);

		// Ignore unknown attrs
		if(mnl_attr_type_valid(attr, RTA_MAX) < 0)
		{
			return MNL_CB_OK;
		}

		if(type == RTA_OIF)
		{
			if(mnl_attr_validate(attr, MNL_TYPE_U32) < 0)
			{
				fmt::print(stderr, "Got invalid OIF attr, ignoring\n");
				return MNL_CB_OK;
			}

			if(outputInterface < 0)
				outputInterface = mnl_attr_get_u32(attr);
		}

		return MNL_CB_OK;
	});

	int ret = mnl_socket_recvfrom(m_mnl.get(), buf, sizeof(buf));
	if(ret > 0)
	{
		mnl_cb_run(buf, ret, seq, portid, &CallbackHelper::callback, &helper);
	}

	// No route found :-(
	if(outputInterface < 0)
		return {};

	char ifname[IF_NAMESIZE+1];
	if(!if_indextoname(outputInterface, ifname))
	{
		fmt::print(stderr, "Got invalid output interface index: {}\n", outputInterface);
		return {};
	}

	return ifname;
}

}
