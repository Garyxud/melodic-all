###############################################################
# Pairing rapp specification
###############################################################

# Unique name that represents this pairing. Note that you can
# have multiple configurations of a single rapp, so the rapp
# name alone is not enough to uniquely identify the pairing
string name

# ros resource name of the rapp, e.g. rocon_apps/talker
string rapp

###############################################################
# Customising the rapp launch instance
###############################################################

# Can load the default rapp description, but sometimes useful to
# override it to provide more human friendly information.
string description

# Again, can load the default rapp icon, but may want to override it.
rocon_std_msgs/Icon icon

rocon_std_msgs/Remapping[] remappings

# Key value pairs representing rapp parameters
rocon_std_msgs/KeyValue[] parameters
