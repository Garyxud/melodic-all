# This expresses a marker/fiducial in a 3D space
int32[] ids                  # possible ids of the detected fiducial (empty if no id could be identified)
float64[] ids_confidence     # confidence of the possible ids in [0, 1]
geometry_msgs/Pose pose      # pose of the detected marker/fiducial
