# Sent by the navigator at 5-10 Hz.

Header 	header

geographic_msgs/GeoPoint global_position
geographic_msgs/GeoPoint origin

# NED position and altitude in metres. 
NED position
float32 altitude

# Body velocities in metres/sec., "water-velocity"
geometry_msgs/Point body_velocity
# Velocities measured wrt the seafloor/ground in metres/sec.
geometry_msgs/Point seafloor_velocity

# Orientation and orientation rate are in radians and radians/sec using RPY
geometry_msgs/Vector3 orientation
geometry_msgs/Vector3 orientation_rate

# Variances for position and orientation
NED position_variance
geometry_msgs/Vector3 orientation_variance

# Status and associated constants (flags)
# These flags are here for legacy reasons.
# We need a wider adoption of this package to rethink flags, get in contact if you have suggestions.
# RESERVED RANGE: 0-100.
uint8 status
uint8 STATUS_FAULT = 0
uint8 STATUS_LOCAL_FRAME_OK = 1
uint8 STATUS_GLOBAL_FRAME_OK = 2
uint8 STATUS_POSITION_OK = 3
uint8 STATUS_ESTIMATION_ERROR_OK  = 8
uint8 STATUS_WATER_VELOCITY_OK = 16
uint8 STATUS_GROUND_VELOCITY_OK = 32
uint8 STATUS_VELOCITY_OK = 48
uint8 STATUS_ALL_OK = 59
# Use custom codes >100
