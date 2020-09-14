############################
# Local magnetic field message for KVH GEO FOG 3D local magnetic field packet
############################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# Local magnetic field X (mG)
#
float32 loc_mag_field_x_mG

#
# Local magnetic field Y (mG)
#
float32 loc_mag_field_y_mG

#
# Local magnetic field Z (mG)
float32 loc_mag_field_z_mG