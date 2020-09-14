### ROCON resource request or response.
#
#   A scheduler_msgs/Request message includes one Resource message for
#   each desired resource.  The scheduler responds using this same
#   message to identify exactly what corresponding resource it has
#   granted.

# This is usually a uniquely identifying ros_package/rapp name
# identifier, which is unique because ros packages are unique.
string rapp

# Unique identifier assigned by the requester to track resources
# assigned in the scheduler feedback.
uuid_msgs/UniqueID id

# Uniform Resource Identifier for the platform.  ROCON defines two
# kinds of URI strings:
#
# 1) A "resource description URI" is a canonical string for a specific
#    device.  All components are fully resolved to their most specific
#    values, like:
#
#      "rocon:/turtlebot/dude3/hydro/precise"
#
#    The scheduler provides fully resolved resource description URIs
#    in its feedback for requests that have been granted.
#
# 2) A "request URI" may provide a similar descriptive representation,
#    or may include patterns for matching multiple alternative
#    platforms.  Omitted or '*' patterns match any valid value:
#
#      ""                               # (empty string): any platform
#      "rocon:/turtlebot"               # any turtlebot
#      "rocon:/(turtlebot|segbot)"      # any robot of either type
#      "rocon:/*/dude3"                 # any device named dude3
#      "rocon:/*/*/ros/ubuntu"          # any ROS Ubuntu platform
#
string uri

# Remappings which get passed on for starting the rapps. Also potentially for
# more esoteric use cases such as checking whether an app is sharable.
rocon_std_msgs/Remapping[] remappings
rocon_std_msgs/KeyValue[] parameters
