# 
# Describes an interactive concert client.
#
# Unique names - human consumable rocon name as well as globally unique name
# provided by the remocon client.

string name
uuid_msgs/UniqueID id

rocon_std_msgs/MasterInfo platform_info

# easy to read display names of interactions running on this remocon
string[] running_interactions
