############################
# North Seeking Initialisation status message for
# KVH GEO FOG 3D north seeking Initialisation status packet
############################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# North Seeking initialisation flags.
# Bits:
#   0 - North seeking initialisation complete
#   1 - North seeking initialisation cannot start because postion unknown
#   2 - North seeking initialisation paused due to excessive rolling
#   3 - North seeking initialisation pauded due to excessive pitching
#   4 - North seeking initialisation paused due to excessive movement
#
uint16 flags

#
# Quadrant 1 data collection progress in percent
#
uint8 quadrant_1_data_per

#
# Quadrant 2 data collection progress in percent
#
uint8 quadrant_2_data_per

#
# Quadrant 3 data collection progress in percent
#
uint8 quadrant_3_data_per

#
# Quadrant 4 data collection progress in percent 
#
uint8 quadrant_4_data_per

#
# Current rotation angle in radians
# -pi/4 to pi/4 - quadrant 1
# pi/4 to 3pi/3 - quadrant 2
# -3pi/4 to 3pi/4 - quadrant 3
# -3pi/4 to -pi/4 - quadrant 4
float32 current_rotation_angle_rad

#
# Current gyroscope bias solution X (rad/s)
#
float32 current_gyro_bias_sol_x_rad_s

#
# Current gyroscope bias solution Y (rad/s)
#
float32 current_gyro_bias_sol_y_rad_s

#
# Current gyroscope bias solution z (rad/s)
#
float32 current_gyro_bias_sol_z_rad_s

#
# Current gyroscope bias solution error (%)
#
float32 current_gyro_bias_sol_error_per
