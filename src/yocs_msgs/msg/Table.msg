# Semantic annotation for a table; by now a clone of column but with different semantics
# In the future we must support also rectangular tables
#  - Orientation is ignored
#  - Z provides the lower border of the column (normally 0)

string  name
float32 radius
float32 height
geometry_msgs/PoseWithCovarianceStamped pose
