##############################
# Detailed satellites message for KVG GEO FOG 3D detailed satellites packet
##############################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header 

#
# Integer enumeration of values
# Values:
#   0 - Unknown
#   1 - GPS
#   2 - GLONASS
#   3 - BeiDou
#   4 - GALILEO
#   5 - SBAS
#   6 - QZSS
#   7 - Starfire
#   8 - Omnistar 
#
uint8[] satellite_system

#
# Not fully familiar, but seems to be an editable id of 
# the satellite
#
uint8[] satellite_number

#
# Each bit represents a different satellite frequency
# Bits:
#   0 - L1 C/A
#   1 - L1 C
#   2 - L1 P
#   3 - L1 M 
#   4 - L2 C 
#   5 - L2 P 
#   6 - L2 M 
#   7 - L5
#
int8[] satellite_frequencies

#
# Measurement of the eleveation angle in degrees.
# Elevation angle is the degree between the horizontal
# plane and the line of sight in the vertical plane.
#
uint8[] elevation_deg

#
# Measurement of azimuth in degrees.
# The azimuth is the angle between a body and North. 
#
uint16[] azimuth_deg

#
# Signal to noise ratio in decibels
#
uint8[] snr_decibal