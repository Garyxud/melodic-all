# Body frame vehicle velocity request.
# Coordinate units are metres/sec and radians/sec.
# NB: Coordinate frame used is ROS standard [forward, left, up],
# to simplify integration with arm control systems.

# header.frame_id should be set to the base_link frame of the vehicle.
Header header

geometry_msgs/Twist twist

# Axes of control to disable, in body frame.
bool[6] disable_axis # x y z r p y
