Header header

# Summary of the client settings
string server_host
uint32 server_port
string source_interface
float32 packet_rate
uint32 packet_size

# Proportion of packets that have latencies within a bin. The first bin
# starts at zero, bins end at the value in latency_bin_values. 
# len(latency_bin_values) = len(latency_bin_limits) + 1
float32[] latency_bin_limits
float32[] latency_bin_values

# Average latency in seconds and loss rate (between 0 and 1) for packets
# that fall within one of the bins.
float32 average_latency_fresh
float32 loss_fresh

# Average latency in seconds for all
# received packets, including very old ones.
float32 average_latency_all
