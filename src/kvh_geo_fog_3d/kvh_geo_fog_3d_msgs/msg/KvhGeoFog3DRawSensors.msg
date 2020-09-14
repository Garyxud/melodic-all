###############################################
# Raw Sensors message for the KVH GEO FOG 3D raw sensors packet
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
# Accelerometer X,Y,Z in m/s^2
#
float32 accelerometer_x_mpss
float32 accelerometer_y_mpss
float32 accelerometer_z_mpss

#
# Gyroscope X,Y,Z rad/s
#
float32 gyro_x_rps
float32 gyro_y_rps
float32 gyro_z_rps

#
# Mangetometer in mG
#
float32 magnetometer_x_mG
float32 magnetometer_y_mG
float32 magnetometer_z_mG

#
# IMU Temperature deg C
#
float32 imu_temp_c

#
# Pressure in pascals
#
float32 pressure_pa

#
# Pressure Temperature deg C
#
float32 pressure_temp_c
