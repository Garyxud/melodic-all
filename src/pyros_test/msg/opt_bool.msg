# We use an array to describe an "optional" field when propagated from non-ROS messages.
# The array will be empty if the field is not present in incoming message
# The array will contain only one element if the field is present in incoming message
bool[] data