// NL80211 interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_NET_MONITOR_NL80211_H
#define NIMBRO_NET_MONITOR_NL80211_H

#include <memory>
#include <optional>

#include "standalone_proto.h"

class NL80211
{
public:
    NL80211();
    ~NL80211();

    NL80211(const NL80211&) = delete;
    NL80211& operator=(const NL80211&) = delete;

    std::optional<int> getPhyByName(const std::string& name);
    std::optional<int> getInterfaceByName(const std::string& name);
    std::optional<int> getPhyByMACAddress(const std::string& addr);
    std::optional<int> getPhyForInterface(const std::string& ifaceName);

    std::optional<std::string> getPhyName(int phy);

    void dumpFreqs(int phy);

    std::optional<proto::WifiStats> getStats(int interface);

private:
    class Private;
    std::unique_ptr<Private> m_d;
};

#endif
