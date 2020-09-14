###############################################################
# Required Specifications
###############################################################

# User friendly version of the name. Useful to be customised
# differently from the name for either 1) branding, or 2) because
# some names are complicated visually (e.g. android names).
string name

# This should be the string required to install and execute
# the interaction from a remocon. 
# - android app : the intent action name (e.g. com.github.rosjava.android_apps.listener.Listener)
# - web app : the url
# - roslaunch : package resource name (e.g. gopher_meta/minimal.launch)
# - rosrun : package resource name (e.g. gopher_meta/enable_motors)
# - global : just the executable (e.g. rocon_master_info)
string command

# A rocon uri string denoting the platforms this interaction
# is compatible with
string compatibility

###############################################################
# Optional Specifications
###############################################################

# The group this solution has configured the interaction for.
string group

# Should be a default for the interaction, but sometimes useful to
# override it to provide more human friendly information.
string description

# Again, should exist a default, but may want to override it.
rocon_std_msgs/Icon icon

# The namespace that this interaction will be associated
# with. Interfaces from the interaction will be automatically pushed
# into this namespace (for concerts it will be typically
# used by the services to push interfaces into /services/_service_name_).
# It is up to the user to make sure this is unique to avoid
# potential conflicts.
string namespace

# Any remappings that need to be applied
rocon_std_msgs/Remapping[] remappings

# Yaml string representing parameters for the interaction
string parameters

# Maximum number of permitted connections (-1 = any)
int32 UNLIMITED_INTERACTIONS = -1
int32 max

# If it should be paired with launch configurations via rapps
bool bringup_pairing
bool teardown_pairing
string[] required_pairings

###############################################################
# Parameters finalised by the interactions manager
###############################################################

# This is a crc32 hash code for the name-group-command
# triple that uniquely identifies this interaction.
# crc32 gets a few collisions, so we should be careful of that.
# It is used by the nfc android auto-launching program so we can
# compactify the request in the url sent by the nfc to the autolauncher.
int32 hash

