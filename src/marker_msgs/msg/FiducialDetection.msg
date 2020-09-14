# This expresses a marker/fiducial detection
Header header                  # timestamp in the header is the acquisition time and frame

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] camera_d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  camera_k # 3x3 row-major matrix

string   type                  # marker type used (bch, artag, mip_36h12, stage, ...)
Fiducial[] fiducial            # detected markers