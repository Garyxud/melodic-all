# This represents a load carrier by its geometry and its pose

string id

# Required
Box outer_dimensions

# Required
Box inner_dimensions

# Optional. If not provided, the rim thickness is computed from the outer and inner dimensions
Rectangle rim_thickness

geometry_msgs/PoseStamped pose

# True if a detected load carrier is overfilled. Ignored on setting load carrier.
bool overfilled
