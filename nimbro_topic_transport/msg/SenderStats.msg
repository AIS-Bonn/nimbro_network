Header header

# Node name
string node

# Protocol ("TCP" or "UDP")
string protocol

# Connection label
string label

# Local hostname
string host

# Statistics for each stream destination
DestinationStats[] destinations

# Bandwidth in bit/s
float32 bandwidth

# Topic specific bandwidth
TopicBandwidth[] topics
