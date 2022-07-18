
bool is_wifi_device

# Association info
bool associated
duration associated_since
string ssid
uint16 frequency_mhz

# Signal strength info
int8 signal_dbm
int8 signal_avg_dbm
int8 beacon_signal_dbm
int8[] signal_per_chain_dbm

# TX Bitrate
uint8 tx_mcs
uint16 tx_bw
uint64 tx_bitrate

# RX Bitrate
uint8 rx_mcs
uint16 rx_bw
uint64 rx_bitrate
