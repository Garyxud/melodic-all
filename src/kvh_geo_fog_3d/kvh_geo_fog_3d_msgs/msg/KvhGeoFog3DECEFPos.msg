############################
# ECEF Position message for KVH GEO FOG 3D ECEF packet
############################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# ECEF X (meters)
#
float64 ecef_x_m

#
# ECEF Y (meters)
#
float64 ecef_y_m

#
# ECEF Z (meters)
#
float64 ecef_z_m