####################################
# UTM Position message for KVH GEO FOG 3D UTM position packet
####################################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# Northing in meters
#
float64 northing_m

#
# Easting in meters
#
float64 easting_m

#
# Height in meters
#
float64 height_m

#
# Zone number
# (Does not seem to exist in the spatial_packets.h header file, only in the documentation)
# uint8 zone_number

#
# Zone character
#
char zone_character