###############################################
# Raw GNSS message for the KVH GEO FOG 3D raw GNSS packet
###############################################

#
# This message contains completely raw data. This is,
# it has not been passed through the KVH kalman filters
#

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# Unix time stamp seconds and microseconds
# To get full time add them together
#
uint32 unix_time_s
uint32 unix_time_us

#
# Longitude and Latitude in radians
#
float64 latitude_rad
float64 longitude_rad

#
# Height in meters
#
float64 height_m

#
# Velocity North, East, and Down in meters
#
float32 vel_north_m
float32 vel_east_m
float32 vel_down_m

#
# Standard deviation lat, long, and height in m
#
float32 latitude_stddev_m
float32 longitude_stddev_m
float32 height_stddev_m

#
# Tilt and Heading in radians
#
float32 tilt_rad
float32 heading_rad

#
# Tilt and Heading std deviation in radians
#
float32 tilt_stddev_rad
float32 heading_stddev_rad

#
# Raw GNSS Status
# Bits 0,1,2 value in binary
# 0 - No GNSS Fix
# 1 - 2D GNSS Fix 
# 2 - 3D GNSS Fix
# 3 - SBAS GNSS Fix
# 4 - Differential GNSS Fix
# 5 - Omnistar/Starfire GNSS Fix
# 6 - RTK Float GNSS Fix
# 7 - RTF Fixed GNSS Fix 
#
# 3 - Doppler velocity valid
# 4 - Time valid
# 5 - External GNSS valid
# 6 - Tilt valid
# 7 - Heading valid
# 8 - Floating ambiguity heading
# 9-16 - reserved (set to zero)
# 
# We seperate these to different variables for ease
#
uint8 gnss_fix
uint8 doppler_velocity_valid
uint8 time_valid
uint8 external_gnss
uint8 tilt_valid
uint8 heading_valid