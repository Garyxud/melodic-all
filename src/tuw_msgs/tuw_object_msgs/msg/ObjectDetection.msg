# This expresses a object detection
Header header                                 # timestamp in the header is the acquisition time and frame

float32 distance_min                          # distance minimum range value [m]
float32 distance_max                          # distance maximum range value [m]
float32 distance_max_id                       # distance maximum range value to detect id [m]

geometry_msgs/Quaternion view_direction       # view direction
float32 fov_horizontal                        # field of view horizontal [rad]
float32 fov_vertical                          # field of view vertical [rad]

string   type                                 # object type used (person, obstacle, ...)

ObjectWithCovariance[] objects                # detected objects (with covariance)

string          sensor_type                      # Sensor / detector type, see example constants below.

# object type constants
string OBJECT_TYPE_PERSON = person
string OBJECT_TYPE_OBSTACLE = obstacle
string OBJECT_TYPE_TRAFFIC_CONE = traffic_cone
string OBJECT_TYPE_DOOR = door

# sensor type constants
string          SENSOR_TYPE_GENERIC_LASER_2D = laser2d
string          SENSOR_TYPE_GENERIC_LASER_3D = laser3d
string          SENSOR_TYPE_GENERIC_MONOCULAR_VISION = mono
string          SENSOR_TYPE_GENERIC_STEREO_VISION = stereo
string          SENSOR_TYPE_GENERIC_RGBD = rgbd
