###############################################
# Satellites message for the KVH GEO FOG 3D satellites packet
###############################################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# Horizontal dilution of precision (HDOP). Describes
# the strength of current satellite configuration.
#
float32 hdop

#
# Vertical dilution of precision (VDOP). Describes
# the strength of current satellite configuration.
#
float32 vdop

#
# Number of total satellites in sight
#
uint8 gps_satellites

#
# Number of glonass satellites in sight
#
uint8 glonass_satellites

#
# Number of beidou satellites in sight
#
uint8 beidou_satellites

#
# Number of galileo satellites in sight
#
uint8 galileo_satellites

#
# Number of sbas satellites in sight
#
uint8 sbas_satellites