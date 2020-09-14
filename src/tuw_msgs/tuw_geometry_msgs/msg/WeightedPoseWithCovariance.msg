# This represents a pose in free space with uncertainty and a scoring weight.

geometry_msgs/Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

# [0, 1] weight of the pose, used for scoring a pose particle
float64 weight
