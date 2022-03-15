// NL80211 interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "nl80211.h"

#include <libmnl/libmnl.h>

#include <netdb.h>
#include <fcntl.h>

#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/nl80211.h>
#include <linux/genetlink.h>

#include <net/if.h>

#include <functional>

#include <fmt/format.h>
#include <fmt/ranges.h>


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

    struct NestedType
    {
        using Type = const nlattr*;
        static constexpr mnl_attr_data_type NLType = MNL_TYPE_NESTED;
        static const nlattr* getter(const nlattr* attr)
        {
            if(mnl_attr_validate(attr, MNL_TYPE_NESTED) < 0)
                throw std::runtime_error{"Expected a NESTED attr"};

            return attr;
        }
    };

    struct MACType
    {
        using Type = std::array<uint8_t, 6>;
        static constexpr mnl_attr_data_type NLType = MNL_TYPE_UNSPEC;
        static Type getter(const nlattr* attr)
        {
            if(mnl_attr_get_payload_len(attr) != 6)
                throw std::runtime_error{"Invalid MAC length"};

            Type ret;
            std::memcpy(ret.data(), mnl_attr_get_payload(attr), ret.size());
            return ret;
        }
    };

    template<int ID>
    struct TypeForIDStruct
    {
    };
    template<> struct TypeForIDStruct<NL80211_ATTR_WIPHY> { using Type = U32Type; };
    template<> struct TypeForIDStruct<NL80211_ATTR_WIPHY_NAME> { using Type = StringType; };
    template<> struct TypeForIDStruct<NL80211_ATTR_IFNAME> { using Type = StringType; };
    template<> struct TypeForIDStruct<NL80211_ATTR_IFINDEX> { using Type = U32Type; };
    template<> struct TypeForIDStruct<NL80211_ATTR_IFTYPE> { using Type = U32Type; };
    template<> struct TypeForIDStruct<NL80211_ATTR_WIPHY_BANDS> { using Type = NestedType; };
    template<> struct TypeForIDStruct<NL80211_ATTR_MAC> { using Type = MACType; };
    template<> struct TypeForIDStruct<NL80211_ATTR_BSS> { using Type = NestedType; };
    template<> struct TypeForIDStruct<NL80211_ATTR_STA_INFO> { using Type = NestedType; };


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
    void stashIntoTuple(const nlattr* attr, uint16_t id, std::tuple<std::optional<typename TypeForID<IDs>::Type>...>& t)
    {
        if constexpr(sizeof ... (IDs) != 0)
        {
            constexpr int ThisID = nth_element<I, IDs...>;
            using Type = TypeForID<ThisID>;
            if(ThisID == id)
            {
                std::get<I>(t) = Type::getter(attr);
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

    struct AttrCallbackHelper
    {
        using AttrCb = std::function<void(const nlattr*)>;

        AttrCallbackHelper(const AttrCb& attrCb)
         : m_attrCb(attrCb)
        {}

        static int attrCallback(const nlattr* attr, void* arg)
        {
            AttrCallbackHelper* helper = reinterpret_cast<AttrCallbackHelper*>(arg);
            helper->m_attrCb(attr);
            return MNL_CB_OK;
        }

        AttrCb m_attrCb;
    };

    template<typename Cb>
    void parseNested(const nlattr* attr, const Cb& cb)
    {
        AttrCallbackHelper helper{cb};
        mnl_attr_parse_nested(attr, &AttrCallbackHelper::attrCallback, &helper);
    }

    int cb_noop(const struct nlmsghdr *nlh, void *data)
    {
        return MNL_CB_OK;
    }

    int cb_error(const struct nlmsghdr *nlh, void *data)
    {
        auto err = reinterpret_cast<const nlmsgerr*>(mnl_nlmsg_get_payload(nlh));

        if (nlh->nlmsg_len < mnl_nlmsg_size(sizeof(struct nlmsgerr)))
        {
            errno = EBADMSG;
            return MNL_CB_ERROR;
        }

        /* Netlink subsystems returns the errno value with different signess */
        if (err->error < 0)
                errno = -err->error;
        else
                errno = err->error;

        if(err->error != 0)
        {
            AttrCallbackHelper helper{[&](const nlattr* attr){
                if(mnl_attr_get_type(attr) == 1) // NLMSGERR_ATTR_MSG, but this is not defined in libmnl's netlink.h
                {
                    fmt::print(stderr, "NETLINK ERROR: {}\n", mnl_attr_get_str(attr));
                }
            }};
            mnl_attr_parse(nlh, sizeof(nlmsgerr), &AttrCallbackHelper::attrCallback, &helper);
        }

        return err->error == 0 ? MNL_CB_STOP : MNL_CB_ERROR;
    }

    int cb_stop(const struct nlmsghdr *nlh, void *data)
    {
        return MNL_CB_STOP;
    }

    class NLException : public std::runtime_error
    {
    public:
        explicit NLException(const std::string& str) : std::runtime_error{str}
        {}
    };
}

class NL80211::Private
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
        std::tuple<std::optional<typename TypeForID<IDs>::Type>...> data;

        CallbackHelper helper{[&](const nlmsghdr* nlh) -> int {
            // Called for each message
            auto* genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_get_payload(nlh));
            if(!genl)
                throw std::logic_error{"Did not get nlh from libmnl"};

            // Clear data tuple
            data = {};

            mnl_attr_parse(nlh, sizeof(*genl), &CallbackHelper::attrCallback, &helper);

            // Call callback on finished data
            std::apply(cb, data);

            return MNL_CB_OK;
        }, [&](const nlattr* attr) -> int {
            auto id = mnl_attr_get_type(attr);

            stashIntoTuple<0, IDs...>(attr, id, data);

            return MNL_CB_OK;
        }};

        mnl_cb_t controlCallbacks[NLMSG_MIN_TYPE] = {};
        controlCallbacks[NLMSG_NOOP]    = cb_noop;
        controlCallbacks[NLMSG_ERROR]   = cb_error;
        controlCallbacks[NLMSG_DONE]    = cb_stop;
        controlCallbacks[NLMSG_OVERRUN] = cb_noop;

        int ret = mnl_socket_recvfrom(mnl.get(), buf, sizeof(buf));
        while(ret > 0)
        {
            ret = mnl_cb_run2(buf, ret, seq, portid, &CallbackHelper::callback, &helper, &controlCallbacks[0], sizeof(controlCallbacks));
            if(ret <= 0)
                break;

            ret = mnl_socket_recvfrom(mnl.get(), buf, sizeof(buf));
        }
        if(ret == -1)
        {
            perror("NL error");
            throw NLException{fmt::format("NL80211 error: {}", strerror(errno))};
        }
    }

    int getFamilyID(const char* familyStr)
    {
        int seq = this->seq++;

        char buf[MNL_SOCKET_BUFFER_SIZE];
        auto nlh = mnl_nlmsg_put_header(buf);
        nlh->nlmsg_type = GENL_ID_CTRL;
        nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        nlh->nlmsg_seq = seq;

        auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
        genl->cmd = CTRL_CMD_GETFAMILY;
        genl->version = 1;

        mnl_attr_put_u16(nlh, CTRL_ATTR_FAMILY_ID, GENL_ID_CTRL);
        mnl_attr_put_strz(nlh, CTRL_ATTR_FAMILY_NAME, familyStr);

        if(mnl_socket_sendto(mnl.get(), nlh, nlh->nlmsg_len) < 0)
        {
            perror("mnl_socket_sendto");
            exit(EXIT_FAILURE);
        }

        int id = -1;

        CallbackHelper helper([&](const nlmsghdr* nlh){
            // Called for each message
            auto* genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_get_payload(nlh));
            if(!genl)
                return MNL_CB_ERROR;

            mnl_attr_parse(nlh, sizeof(*genl), &CallbackHelper::attrCallback, &helper);

            return MNL_CB_OK;
        }, [&](const nlattr* attr){
            // Called for each attribute
            int type = mnl_attr_get_type(attr);

            // Ignore unknown attrs
            if(mnl_attr_type_valid(attr, CTRL_ATTR_MAX) < 0)
            {
                fprintf(stderr, "Ignoring unknown attr\n");
                return MNL_CB_OK;
            }

            switch(type)
            {
                case CTRL_ATTR_FAMILY_ID:
                    if(mnl_attr_validate(attr, MNL_TYPE_U16) < 0)
                        return MNL_CB_ERROR;

                    id = mnl_attr_get_u16(attr);
                    return MNL_CB_OK;
            }

            return MNL_CB_OK;
        });

        int ret = mnl_socket_recvfrom(mnl.get(), buf, sizeof(buf));
        while(ret > 0)
        {
            ret = mnl_cb_run(buf, ret, seq, portid, &CallbackHelper::callback, &helper);
            if(ret <= 0)
                break;

            ret = mnl_socket_recvfrom(mnl.get(), buf, sizeof(buf));
        }
        if(ret == -1)
        {
            perror("Could not receive NL message");
            std::exit(EXIT_FAILURE);
        }

        if(id == -1)
        {
            fprintf(stderr, "Did not receive ID answer\n");
            std::exit(EXIT_FAILURE);
        }

        return id;
    }

    std::shared_ptr<mnl_socket> mnl;
    unsigned int seq = 0;
    unsigned int portid = 0;

    int familyID = 0;
};

NL80211::NL80211()
 : m_d{std::make_unique<Private>()}
{
    auto deleter = std::bind(mnl_socket_close, std::placeholders::_1);
    m_d->mnl.reset(mnl_socket_open(NETLINK_GENERIC), deleter);

    if(mnl_socket_bind(m_d->mnl.get(), 0, MNL_SOCKET_AUTOPID) < 0)
    {
        perror("mnl_socket_bind");
        exit(EXIT_FAILURE);
    }
    m_d->portid = mnl_socket_get_portid(m_d->mnl.get());

    m_d->familyID = m_d->getFamilyID(NL80211_GENL_NAME);
}

NL80211::~NL80211()
{
}

std::optional<int> NL80211::getPhyByMACAddress(const std::string& addr)
{
    char buf[MNL_SOCKET_BUFFER_SIZE];
    auto nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = m_d->familyID;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

    auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
    genl->cmd = NL80211_CMD_GET_WIPHY;
    genl->version = 0;

    std::optional<int> phyIdx;

    m_d->communicate<NL80211_ATTR_WIPHY, NL80211_ATTR_WIPHY_NAME>(nlh, [&](auto thisPhyIdx, auto thisPhyName){
        if(!thisPhyIdx || !thisPhyName)
            return;

        int fd = open(fmt::format("/sys/class/ieee80211/{}/macaddress", *thisPhyName).c_str(), O_RDONLY);
        if(!fd)
            return;

        char macbuf[128];

        int ret = read(fd, macbuf, sizeof(macbuf)-1);
        close(fd);

        if(ret <= 0)
            return;

        macbuf[ret] = 0;

        // Strip trailing \n from syfs
        if(ret > 0 && macbuf[ret-1] == '\n')
            macbuf[ret-1] = 0;

        if(strcasecmp(addr.c_str(), macbuf) == 0)
            phyIdx = *thisPhyIdx;
    });

    return phyIdx;
}

std::optional<int> NL80211::getPhyByName(const std::string& name)
{
    char buf[MNL_SOCKET_BUFFER_SIZE];
    auto nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = m_d->familyID;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

    auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
    genl->cmd = NL80211_CMD_GET_WIPHY;
    genl->version = 0;

    std::optional<int> phyIdx;

    m_d->communicate<NL80211_ATTR_WIPHY_NAME, NL80211_ATTR_WIPHY>(nlh, [&](auto thisPhyName, auto thisPhyIdx){
        if(!thisPhyName || !thisPhyIdx)
            return;

        if(*thisPhyName == name)
            phyIdx = thisPhyIdx;
    });

    return phyIdx;
}

std::optional<int> NL80211::getInterfaceByName(const std::string& name)
{
    char buf[MNL_SOCKET_BUFFER_SIZE];
    auto nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = m_d->familyID;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

    auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
    genl->cmd = NL80211_CMD_GET_INTERFACE;
    genl->version = 0;

    std::optional<int> ifaceIdx;

    m_d->communicate<NL80211_ATTR_IFNAME, NL80211_ATTR_IFINDEX>(nlh, [&](auto thisIfaceName, auto thisIfaceIdx){
        if(!thisIfaceName || !thisIfaceIdx)
            return;

        if(*thisIfaceName == name)
            ifaceIdx = thisIfaceIdx;
    });

    return ifaceIdx;
}

std::optional<int> NL80211::getPhyForInterface(const std::string& ifaceName)
{
    char buf[MNL_SOCKET_BUFFER_SIZE];
    auto nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = m_d->familyID;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

    auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
    genl->cmd = NL80211_CMD_GET_INTERFACE;
    genl->version = 0;

    std::optional<int> phyIdx;

    m_d->communicate<NL80211_ATTR_IFNAME, NL80211_ATTR_WIPHY>(nlh, [&](auto ifname, auto wiphy){
        if(!ifname || !wiphy)
            return;

        if(*ifname == ifaceName)
            phyIdx = wiphy;
    });

    return phyIdx;
}

std::optional<std::string> NL80211::getPhyName(int phy)
{
    char buf[MNL_SOCKET_BUFFER_SIZE];
    auto nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = m_d->familyID;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;

    auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
    genl->cmd = NL80211_CMD_GET_WIPHY;
    genl->version = 0;

    mnl_attr_put_u32(nlh, NL80211_ATTR_WIPHY, phy);

    std::optional<std::string> phyName;

    m_d->communicate<NL80211_ATTR_WIPHY_NAME>(nlh, [&](auto wiphyName){
        phyName = wiphyName;
    });

    return phyName;
}

void NL80211::dumpFreqs(int phy)
{
    char buf[MNL_SOCKET_BUFFER_SIZE];
    auto nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = m_d->familyID;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

    auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
    genl->cmd = NL80211_CMD_GET_WIPHY;
    genl->version = 0;

    mnl_attr_put_u32(nlh, NL80211_ATTR_WIPHY, phy);
    mnl_attr_put(nlh, NL80211_ATTR_SPLIT_WIPHY_DUMP, 0, nullptr);

    fmt::print("Available frequencies:\n");

    m_d->communicate<NL80211_ATTR_WIPHY, NL80211_ATTR_WIPHY_BANDS>(nlh, [&](auto wiphy, auto bands){
        if(!bands || !wiphy)
            return;

        if(*wiphy != phy)
            return;

        parseNested(*bands, [&](const nlattr* band){
            if(mnl_attr_validate(band, MNL_TYPE_NESTED) < 0)
            {
                fmt::print(stderr, "Unknown band type\n");
                return;
            }

            parseNested(band, [&](const nlattr* band_attr){
                int type = mnl_attr_get_type(band_attr);

                switch(type)
                {
                    case NL80211_BAND_ATTR_FREQS:
                    {
                        parseNested(band_attr, [&](const nlattr* freq){
                            if(mnl_attr_validate(band, MNL_TYPE_NESTED) < 0)
                            {
                                fmt::print(stderr, "Unknown freq type\n");
                                return;
                            }

                            fmt::print("Frequency:");

                            parseNested(freq, [&](const nlattr* f){
                                switch(static_cast<nl80211_frequency_attr>(mnl_attr_get_type(f)))
                                {
                                    case NL80211_FREQUENCY_ATTR_FREQ:
                                        fmt::print(" {}MHz", mnl_attr_get_u32(f));
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_10MHZ:
                                        fmt::print(" No10MHz");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_20MHZ:
                                        fmt::print(" No20MHz");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_80MHZ:
                                        fmt::print(" No80MHz");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_160MHZ:
                                        fmt::print(" No160MHz");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_DISABLED:
                                        fmt::print(" Disabled");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_IR:
                                        fmt::print(" NoIR");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_MAX_TX_POWER:
                                        fmt::print(" TX={}mBm", mnl_attr_get_u32(f));
                                        break;
                                    case NL80211_FREQUENCY_ATTR_RADAR:
                                        fmt::print(" Radar");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_DFS_STATE:
                                        fmt::print(" DFS");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_HT40_MINUS:
                                        fmt::print(" NoHT40-");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_NO_HT40_PLUS:
                                        fmt::print(" NoHT40+");
                                        break;
                                    case NL80211_FREQUENCY_ATTR_INDOOR_ONLY:
                                        fmt::print(" Indoor");
                                        break;
                                    default:
                                        break;
                                }
                            });

                            fmt::print("\n");
                        });
                        break;
                    }
                }
            });
        });
    });
}

std::optional<proto::WifiStats> NL80211::getStats(int iface)
{
    struct BSS_Info
    {
        std::vector<uint8_t> mac;
        std::string ssid;
        uint32_t frequency = 0;
    };

    std::optional<BSS_Info> associatedBSS;

    {
        char buf[MNL_SOCKET_BUFFER_SIZE];
        auto nlh = mnl_nlmsg_put_header(buf);
        nlh->nlmsg_type = m_d->familyID;
        nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_DUMP;

        auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
        genl->cmd = NL80211_CMD_GET_SCAN;
        genl->version = 0;

        mnl_attr_put_u32(nlh, NL80211_ATTR_IFINDEX, iface);

        m_d->communicate<NL80211_ATTR_BSS>(nlh, [&](auto bss){
            if(!bss)
                return;

            BSS_Info info;
            bool assoc = false;

            parseNested(*bss, [&](const nlattr* attr){
                int type = mnl_attr_get_type(attr);

                switch(static_cast<nl80211_bss>(type))
                {
                    case NL80211_BSS_STATUS:
                        assoc = (mnl_attr_get_u32(attr) == NL80211_BSS_STATUS_ASSOCIATED);
                        break;
                    case NL80211_BSS_FREQUENCY:
                        info.frequency = mnl_attr_get_u32(attr);
                        break;
                    case NL80211_BSS_BSSID:
                        info.mac.resize(mnl_attr_get_payload_len(attr));
                        std::memcpy(info.mac.data(), mnl_attr_get_payload(attr), info.mac.size());
                        break;
                    case NL80211_BSS_INFORMATION_ELEMENTS:
                    {
                        const uint8_t* ie = reinterpret_cast<uint8_t*>(mnl_attr_get_payload(attr));
                        int ielen = mnl_attr_get_payload_len(attr);

                        // Find SSID
                        while(ielen >= 2 && ielen - 2 >= ie[1])
                        {
                            if(ie[0] == 0) // SSID IE
                            {
                                uint8_t len = ie[1];
                                const char* data = reinterpret_cast<const char*>(ie + 2);
                                info.ssid = std::string(data, len);
                            }

                            ielen -= ie[1] + 2;
                            ie += ie[1] + 2;
                        }
                        break;
                    }
                    default:
                        break;
                }
            });

            if(assoc)
                associatedBSS = std::move(info);
        });
    }

    if(!associatedBSS)
    {
        proto::WifiStats stats;
        stats.is_wifi_device = true;
        return stats;
    }

    proto::WifiStats stats;
    stats.is_wifi_device = true;
    stats.rx_bw = 20;
    stats.tx_bw = 20;
    stats.associated = true;
    stats.ssid = associatedBSS->ssid;
    stats.frequency_mhz = associatedBSS->frequency;

    {
        char buf[MNL_SOCKET_BUFFER_SIZE];
        auto nlh = mnl_nlmsg_put_header(buf);
        nlh->nlmsg_type = m_d->familyID;
        nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;

        auto genl = reinterpret_cast<genlmsghdr*>(mnl_nlmsg_put_extra_header(nlh, sizeof(genlmsghdr)));
        genl->cmd = NL80211_CMD_GET_STATION;
        genl->version = 0;

        mnl_attr_put_u32(nlh, NL80211_ATTR_IFINDEX, iface);
        mnl_attr_put(nlh, NL80211_ATTR_MAC, associatedBSS->mac.size(), associatedBSS->mac.data());

        m_d->communicate<NL80211_ATTR_STA_INFO>(nlh, [&](auto sta_info){
            if(!sta_info)
                return;

            parseNested(*sta_info, [&](const nlattr* attr){
                switch(mnl_attr_get_type(attr))
                {
                    case NL80211_STA_INFO_RX_BITRATE:
                    {
                        int bitrate32 = 0;
                        int bitrate16 = 0;
                        parseNested(attr, [&](const nlattr* rate_attr){
                            switch(mnl_attr_get_type(rate_attr))
                            {
                                case NL80211_RATE_INFO_BITRATE32:
                                    bitrate32 = mnl_attr_get_u32(rate_attr) * 100ULL * 1000ULL;
                                    break;
                                case NL80211_RATE_INFO_BITRATE:
                                    bitrate16 = mnl_attr_get_u16(rate_attr) * 100ULL * 1000ULL;
                                    break;
                                case NL80211_RATE_INFO_VHT_MCS:
                                    stats.rx_mcs = mnl_attr_get_u8(rate_attr);
                                    break;
								case NL80211_RATE_INFO_160_MHZ_WIDTH:
								case NL80211_RATE_INFO_80P80_MHZ_WIDTH:
                                    stats.rx_bw = 160;
                                    break;
                                case NL80211_RATE_INFO_80_MHZ_WIDTH:
                                    stats.rx_bw = 80;
                                    break;
                                case NL80211_RATE_INFO_40_MHZ_WIDTH:
                                    stats.rx_bw = 40;
                                    break;
                            }
                        });
                        stats.rx_bitrate = bitrate32 ? bitrate32 : bitrate16;
                        break;
                    }
                    case NL80211_STA_INFO_TX_BITRATE:
                    {
                        int bitrate32 = 0;
                        int bitrate16 = 0;
                        parseNested(attr, [&](const nlattr* rate_attr){
                            switch(mnl_attr_get_type(rate_attr))
                            {
                                case NL80211_RATE_INFO_BITRATE32:
                                    bitrate32 = mnl_attr_get_u32(rate_attr) * 100ULL * 1000ULL;
                                    break;
                                case NL80211_RATE_INFO_BITRATE:
                                    bitrate16 = mnl_attr_get_u16(rate_attr) * 100ULL * 1000ULL;
                                    break;
                                case NL80211_RATE_INFO_VHT_MCS:
                                    stats.tx_mcs = mnl_attr_get_u8(rate_attr);
                                    break;
								case NL80211_RATE_INFO_160_MHZ_WIDTH:
								case NL80211_RATE_INFO_80P80_MHZ_WIDTH:
                                    stats.tx_bw = 160;
                                    break;
                                case NL80211_RATE_INFO_80_MHZ_WIDTH:
                                    stats.tx_bw = 80;
                                    break;
                                case NL80211_RATE_INFO_40_MHZ_WIDTH:
                                    stats.tx_bw = 40;
                                    break;
                            }
                        });
                        stats.tx_bitrate = bitrate32 ? bitrate32 : bitrate16;
                        break;
                    }
                    case NL80211_STA_INFO_SIGNAL:
                        stats.signal_dbm = mnl_attr_get_u8(attr);
                        break;
                    case NL80211_STA_INFO_SIGNAL_AVG:
                        stats.signal_avg_dbm = mnl_attr_get_u8(attr);
                        break;
                    case NL80211_STA_INFO_BEACON_SIGNAL_AVG:
                        stats.beacon_signal_dbm = mnl_attr_get_u8(attr);
                        break;
                    case NL80211_STA_INFO_CONNECTED_TIME:
                        stats.associated_since = mnl_attr_get_u32(attr);
                        break;
                }
            });
        });
    }

    return stats;
}

