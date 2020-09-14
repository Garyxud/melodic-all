# Holds the data presented on the statistics tab of SwOS web interface
# All arrays have one entry per enabled port.

Header header
string[] port_names
uint8[] port_numbers
int64[] rx_rate_bytes
int64[] rx_rate_packets
int64[] tx_rate_bytes
int64[] tx_rate_packets
int64[] rx_bytes
int64[] rx_packets
int64[] rx_unicasts
int64[] rx_broadcasts
int64[] rx_multicasts
int64[] rx_64
int64[] rx_127
int64[] rx_255
int64[] rx_511
int64[] rx_1023
int64[] rx_1518
int64[] rx_jumbo
int64[] tx_bytes
int64[] tx_packets
int64[] tx_unicasts
int64[] tx_broadcasts
int64[] tx_multicasts
int64[] tx_64
int64[] tx_127
int64[] tx_255
int64[] tx_511
int64[] tx_1023
int64[] tx_1518
int64[] tx_jumbo