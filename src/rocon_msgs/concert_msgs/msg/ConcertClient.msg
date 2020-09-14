
# The concert alias
string name

# The unfriendly gateway name with the uuid suffix.
string gateway_name

# Platform information about the concert client.
rocon_std_msgs/MasterInfo platform_info

# Is on the same machine ip as the conductor
bool is_local_client

# State of the concert client, see ConcertClientState.msg for string constants
# PENDING, BUSY, BLOCKING, BAD, JOINING, UNINVITED, AVAILABLE, MISSING, GONE 
string state

# Statistics
string ip
gateway_msgs/ConnectionStatistics conn_stats
# time last_connection_timestamp

rocon_app_manager_msgs/Rapp[] rapps
