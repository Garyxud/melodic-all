# single marking on the track

# position of the marking in space, as well as normal angle along the racetrack path
geometry_msgs/Pose2D pose

# type of track marking
uint8 type

uint8 TRACK_MARKING_TYPE_UNKNOWN = 0
uint8 TRACK_MARKING_TYPE_LEFT = 1
uint8 TRACK_MARKING_TYPE_RIGHT = 2
uint8 TRACK_MARKING_TYPE_START_STOP = 4
