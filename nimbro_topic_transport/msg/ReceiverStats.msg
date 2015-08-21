Header header

# Node name
string node

# Protocol ("TCP" or "UDP")
string protocol

# Connection label
string label

# Local hostname
string host

# Remote hostname (if resolvable, otherwise IP)
string remote

# Port
uint16 local_port

# Port
uint16 remote_port

# is Forward Error Correction enabled?
bool fec

# Bandwidth in bit/s
float32 bandwidth

# Drop rate (estimated, percentage of missing packets in a fixed time interval)
float32 drop_rate
