# This expresses a known detected fiducial in image space
int32[] ids                         # possible ids of the detected fiducial (empty if no id could be identified)
float64[] ids_confidence            # confidence of the possible ids in [0, 1]
geometry_msgs/Point[] object_points # Array of object points in the object coordinate space.
geometry_msgs/Point[] image_points  # Array of corresponding image points. Point.z is not used.