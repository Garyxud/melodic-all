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
# Odometer Pulse Count
# Counts number of ticks recieved from the odometer 
#
int32 odometer_pulse_count

#
# Odometer distance
# Shows distance travelled, which is num_ticks * distance_per_tick
#
float32 odometer_distance_m

#
# Odometer Speed
#
float32 odometer_speed_mps

#
# Odometer Slip
#
float32 odometer_slip_m

#
# Odometer Active
# Documentation unclear, assuming 0 is inactive, and non-zero is active
#
uint8 odometer_active