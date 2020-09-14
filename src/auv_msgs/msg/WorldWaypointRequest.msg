# World frame (absolute) waypoint request to pilot.

Header header

# Waypoint request
auv_msgs/NED position
geometry_msgs/Vector3 orientation 

# Axes of control to disable, in body frame.
bool[6] disable_axis # x y z r p y

# Tolerances are in body frame, zero indicates pilot default should be used.
geometry_msgs/Vector3 position_tolerance
geometry_msgs/Vector3 orientation_tolerance

# Capping speeds in 6DOF
geometry_msgs/Twist twist_limit
