# Constants - service types
string TYPE_ROSLAUNCH = roslaunch
string TYPE_CUSTOM    = custom
string TYPE_SHADOW    = shadow

# Pre-configured parameters
string                      resource_name
string                      name
string                      description
string                      author
uint16                      priority
rocon_std_msgs/Icon         icon
string                      launcher_type
string                      launcher
string                      interactions
string                      parameters
rocon_std_msgs/KeyValue[]   parameters_detail

# Initialised parameters (on startup)
uuid_msgs/UniqueID          uuid

# Runtime variables
int16                       status
bool                        enabled
