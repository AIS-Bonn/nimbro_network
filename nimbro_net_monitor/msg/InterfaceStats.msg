
# Name of the network interface
string interface_name

# Max. available bandwidth (e.g. ethernet link speed)
# This may be 0 if the bandwidth is adaptive, for example for WiFi interfaces.
uint64 max_bits_per_second

# Duplex?
bool duplex

# Total used TX Bandwidth in bit/s
float64 tx_bandwidth

# Total used RX Bandwidth in bit/s
float64 rx_bandwidth

WifiStats wifi

PeerStats[] peers
