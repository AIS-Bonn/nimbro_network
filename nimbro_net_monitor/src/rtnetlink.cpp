// RTNetlink interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "rtnetlink.h"

#include <libmnl/libmnl.h>

#include <netdb.h>

#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/genetlink.h>

#include <net/if.h>

#include <functional>

#include <fmt/format.h>

namespace
{
	struct U16Type
	{
		using Type = int;
		static constexpr mnl_attr_data_type NLType = MNL_TYPE_U16;
		static int getter(const nlattr* attr)
		{
			if(mnl_attr_validate(attr, MNL_TYPE_U16) < 0)
				throw std::runtime_error{"Expected an U16 attr"};

			return mnl_attr_get_u16(attr);
		}
	};

	struct U32Type
	{
		using Type = int;
		static constexpr mnl_attr_data_type NLType = MNL_TYPE_U32;
		static int getter(const nlattr* attr)
		{
			if(mnl_attr_validate(attr, MNL_TYPE_U32) < 0)
				throw std::runtime_error{"Expected an U32 attr"};

			return mnl_attr_get_u32(attr);
		}
	};

	struct StringType
	{
		using Type = std::string;
		static constexpr mnl_attr_data_type NLType = MNL_TYPE_STRING;
		static std::string getter(const nlattr* attr)
		{
			if(mnl_attr_validate(attr, MNL_TYPE_STRING) < 0)
				throw std::runtime_error{"Expected an STRING attr"};

			return mnl_attr_get_str(attr);
		}
	};

	struct StatsType
	{
		using Type = rtnl_link_stats64;
		static constexpr mnl_attr_data_type NLType = MNL_TYPE_BINARY;
		static rtnl_link_stats64& getter(const nlattr* attr)
		{
			if(mnl_attr_validate(attr, MNL_TYPE_BINARY) < 0)
				throw std::runtime_error{"Expected a binary attr"};

			if(mnl_attr_get_payload_len(attr) != sizeof(rtnl_link_stats64))
				throw std::runtime_error{"Wrong stats size"};

			return *reinterpret_cast<rtnl_link_stats64*>(mnl_attr_get_payload(attr));
		}
	};

	template<int ID>
	struct TypeForIDStruct
	{
	};
	template<> struct TypeForIDStruct<IFLA_IFNAME> { using Type = StringType; };
	template<> struct TypeForIDStruct<IFLA_STATS64> { using Type = StatsType; };


	template<int ID>
	using TypeForID = typename TypeForIDStruct<ID>::Type;

	template <std::size_t I, int T, int ...Ts>
	struct nth_element_impl {
		static constexpr int value = nth_element_impl<I-1, Ts...>::value;
	};

	template <int T, int ...Ts>
	struct nth_element_impl<0, T, Ts...> {
		static constexpr int value = T;
	};

	template <std::size_t I, int ...Ts>
	constexpr int nth_element = nth_element_impl<I, Ts...>::value;

	template<size_t I = 0, int ... IDs>
	void stashIntoTuple(const nlattr* attr, uint16_t id, std::tuple<ifinfomsg*, std::optional<typename TypeForID<IDs>::Type>...>& t)
	{
		if constexpr(sizeof ... (IDs) != 0)
		{
			constexpr int ThisID = nth_element<I, IDs...>;
			using Type = TypeForID<ThisID>;
			if(ThisID == id)
			{
				std::get<I+1>(t) = Type::getter(attr);
			}
			else
			{
				if constexpr(I+1 != sizeof...(IDs))
					stashIntoTuple<I+1, IDs...>(attr, id, t);
			}
		}
	}

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

class RTNetlink::Private
{
public:
	template<int ... IDs, typename Cb>
	void communicate(nlmsghdr* msg, const Cb& cb)
	{
		int seq = this->seq++;

		msg->nlmsg_seq = seq;

		if(mnl_socket_sendto(mnl.get(), msg, msg->nlmsg_len) < 0)
		{
			perror("mnl_socket_sendto");
			exit(EXIT_FAILURE);
		}

		char buf[MNL_SOCKET_BUFFER_SIZE];
		std::tuple<ifinfomsg*, std::optional<typename TypeForID<IDs>::Type>...> data;

		CallbackHelper helper{[&](const nlmsghdr* nlh) -> int {
			// Called for each message
			auto* info = reinterpret_cast<ifinfomsg*>(mnl_nlmsg_get_payload(nlh));
			if(!info)
				throw std::logic_error{"Did not get nlh from libmnl"};

			// Clear data tuple
			data = {};
			std::get<0>(data) = info;

			mnl_attr_parse(nlh, sizeof(*info), &CallbackHelper::attrCallback, &helper);

			// Call callback on finished data
			std::apply(cb, data);

			return MNL_CB_OK;
		}, [&](const nlattr* attr) -> int {
			auto id = mnl_attr_get_type(attr);

			stashIntoTuple<0, IDs...>(attr, id, data);

			return MNL_CB_OK;
		}};

		int ret = 0;
		bool changed = false;
		while(true)
		{
			ret = mnl_socket_recvfrom(mnl.get(), buf, sizeof(buf));
			if(ret < 0)
			{
				// Blind retry
				if(errno == EINTR)
					continue;

				// Report error
				break;
			}

			// Nothing received, stop.
			if(ret == 0)
				break;

			ret = mnl_cb_run(buf, ret, seq, portid, &CallbackHelper::callback, &helper);
			if(ret < 0 && errno == EINTR)
			{
				// A dump changed while we were retrieving it. We process the rest of the readback, but raise an exception below.
				changed = true;
				continue;
			}

			if(ret <= 0)
				break;
		}

		if(changed)
			throw RetryException{"NL80211 dump changed during retrieval"};

		if(ret == -1)
		{
			perror("NL error");
			throw NLException{fmt::format("NL80211 error: {}", strerror(errno))};
		}
	}

	std::shared_ptr<mnl_socket> mnl;
	unsigned int seq = 0;
	unsigned int portid = 0;
};

RTNetlink::RTNetlink()
 : m_d{std::make_unique<Private>()}
{
	auto deleter = std::bind(mnl_socket_close, std::placeholders::_1);
	m_d->mnl.reset(mnl_socket_open(NETLINK_ROUTE), deleter);

	if(mnl_socket_bind(m_d->mnl.get(), 0, MNL_SOCKET_AUTOPID) < 0)
	{
		perror("mnl_socket_bind");
		exit(EXIT_FAILURE);
	}
	m_d->portid = mnl_socket_get_portid(m_d->mnl.get());
}

RTNetlink::~RTNetlink()
{
}

std::optional<int> RTNetlink::getLinkByName(const std::string& name)
{
	char buf[MNL_SOCKET_BUFFER_SIZE];
	auto nlh = mnl_nlmsg_put_header(buf);
	nlh->nlmsg_type = RTM_GETLINK;
	nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;

	auto rt = reinterpret_cast<rtgenmsg*>(mnl_nlmsg_put_extra_header(nlh, sizeof(struct rtgenmsg)));
	rt->rtgen_family = AF_PACKET;

	mnl_attr_put_strz(nlh, IFLA_IFNAME, name.c_str());

	std::optional<int> retData;

	m_d->communicate<>(nlh, [&](ifinfomsg* info){
		retData = info->ifi_index;
	});

	return retData;
}

std::map<std::string, RTNetlink::Stats> RTNetlink::getLinkStats()
{
	char buf[MNL_SOCKET_BUFFER_SIZE];
	auto nlh = mnl_nlmsg_put_header(buf);
	nlh->nlmsg_type = RTM_GETLINK;
	nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

	auto rt = reinterpret_cast<rtgenmsg*>(mnl_nlmsg_put_extra_header(nlh, sizeof(struct rtgenmsg)));
	rt->rtgen_family = AF_UNSPEC;

	std::map<std::string, Stats> retData;

	m_d->communicate<IFLA_IFNAME, IFLA_STATS64>(nlh, [&](ifinfomsg* info, auto ifname, auto stats){
		if(!stats || !ifname)
			return;

		Stats& st = retData[*ifname];
		st.rx_bytes = stats->rx_bytes;
		st.tx_bytes = stats->tx_bytes;
		st.rx_packets = stats->rx_packets;
		st.tx_packets = stats->tx_packets;
	});

	return retData;
}
