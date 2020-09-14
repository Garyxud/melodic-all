Header header

# Camera information for the left camera
sensor_msgs/CameraInfo left_info

# Camera information for the right camera
sensor_msgs/CameraInfo right_info

# Disparity-to-depth mapping matrix in 4x4 row-major format. Use this
# matrix together with the  Reconstruct3D class from libvisiontransfer
# to transform a disparity map to 3D points.
float64[16] Q

# Translation vector between the coordinate systems of the left and
# right camera.
float64[3] T_left_right

# Rotation matrix between the coordinate systems of the left and right
# camera in 3x3 row-major format.
float64[9] R_left_right
