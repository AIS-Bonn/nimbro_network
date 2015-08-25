Header header

# Node name
string node

# Protocol ("TCP" or "UDP")
string protocol

# Connection label
string label

# Local hostname
string host

# Destination address
string destination

# Destination port
uint16 destination_port

# Source port
uint16 source_port

# Is Forward Error Correction enabled?
bool fec

# Bandwidth in bit/s
float32 bandwidth

# Topic specific bandwidth
TopicBandwidth[] topics
