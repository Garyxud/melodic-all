############################################################################
# System Status message for the KVH GEO FOG 3D
#
# All fields with units of measure are accompanied by suffixes listed here.
# These fields mirror exactly what comes out of the KVH INS, and thus
# may or may not conform to ROS REP 103.
#
# Suffixes used in this file:
#  s         - seconds
#  us        - microseconds
#  rad       - radians
#  m         - meters
#  mps       - meters per second
#  g         - g-force
#  rad_per_s - radians per second
#
# The coordinate frame in header.frame_id is centered at the center base of
# the sensor, with X pointing forward, Y pointing right, and Z pointing
# downward. All values are with respect to this coordinate frame.
# Notably, this does not comply with ROS REP 105, but is standard practice
# amongs localization system providers.
############################################################################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# System status bitmask, as reported by the KVH GEO FOG 3D.
# Each bit is a boolean representing a specific subsystem.
# Bits:
#  0  - System Failure
#  1  - Accelerometer sensor failure
#  2  - Gyroscope sensor failure
#  3  - Magnetometer sensor failure
#  4  - Pressure sensor failure
#  5  - GNSS failure
#  6  - Accelerometer over range
#  7  - Gyroscope over range
#  8  - Magnetometer over range
#  9  - Pressure over range
#  10 - Minimum temperature alarm
#  11 - Maximum temperature alarm
#  12 - Low voltage alarm
#  13 - High voltage alarm
#  14 - GNSS antenna disconnected
#  15 - Data output overflow alarm
#
uint16 system_status

#
# Filter status bitmask, as reported by the KVH GEO FOG 3D.
# Each bit is a boolean representing a specific subsystem.
# Bits:
#  0    - Orientation filter initialized
#  1    - Navigation filter initialized
#  2    - Heading initialized
#  3    - UTC Time initialized
#  4-6  - GNSS fix status (see below)
#  7    - Event 1 occurred
#  8    - Event 2 occurred
#  9    - Internal GNSS enabled
#  10   - Dual antenna heading active
#  11   - Velocity heading enabled
#  12   - Atmospheric altitude enabled
#  13   - External position active
#  14   - External velocity active
#  15   - External heading active
#
# GNSS fix status values (3 bits represented as unsigned integer):
#  0 - No GNSS fix
#  1 - 2D GNSS fix
#  2 - 3D NGSS fix
#  3 - SBAS GNSS fix
#  4 - Differential GNSS fix
#  5 - Omnistar/Starfire GNSS fix
#  6 - RTK Float GNSS fix
#  7 - RTK Fixed GNSS fix
#
uint16 filter_status

#
# UTC time in seconds since January 1st, 1970, including leap seconds
#
uint32 unix_time_s

#
# Sub-second component of time, represented as microseconds since the last
# second. [0,999999]
#
uint32 unix_time_us

#
# Latitude, in radians
#
float64 latitude_rad

#
# Longitude, in radians
#
float64 longitude_rad

#
# Height above WGS84 ellipsoid, in meters.
#
float64 height_m

#
# Velocity with respect to true north, in meters per second
#
float32 absolute_velocity_north_mps

#
# Eastward velocity in the right-handed coordinate frame northward pointing towards
# true north, in meters per second
#
float32 absolute_velocity_east_mps

#
# Downward velocity in the right-handed coordinate frame northward pointing towards
# true north, in meters per second
#
float32 absolute_velocity_down_mps

#
# If mounted to the vehicle rigidly with the suggested orientation from KVH
# (X points forward, Y points right, Z points down), then body_acceleration_x
# is the acceleration front to back, with positive (+) values meaning vehicle
# acceleration forward.
#
float32 body_acceleration_x_mps

#
# If mounted to the vehicle rigidly with the suggested orientation from KVH
# (X points forward, Y points right, Z points down), then body_acceleration_y
# is the acceleration sideways, with positive (+) values meaning vehicle
# accelerating to the right.
#
float32 body_acceleration_y_mps

#
# If mounted to the vehicle rigidly with the suggested orientation from KVH
# (X points forward, Y points right, Z points down), then body_acceleration_z
# is the acceleration up and down, with positive (+) values meaning vehicle
# acceleration downward.
#
float32 body_acceleration_z_mps

#
# Acceleration on the system, represented as g-force.
#
float32 g_force_g

#
# Roll of the system off of gravitational level, in radians
# [-pi,pi)
#
float32 roll_rad

#
# Pitch of the system off of gravitational level, in radians
# [-pi,pi)
#
float32 pitch_rad

#
# Heading angle of the system with respect to true north, in radians
# [0,2*pi)
#
float32 heading_rad

#
# Angular velocity of the system about the X axis, in radians per second.
#
float32 angular_velocity_x_rad_per_s

#
# Angular velocity of the system about the Y axis, in radians per second.
#
float32 angular_velocity_y_rad_per_s

#
# Angular velocity of the system about the Z axis, in radians per second.
#
float32 angular_velocity_z_rad_per_s

#
# Standard deviation of latitude, in meters
#
float32 latitude_stddev_m

#
# Standard deviation of longitude, in meters
#
float32 longitude_stddev_m

#
# Standard deviation of height, in meters
#
float32 height_stddev_m
