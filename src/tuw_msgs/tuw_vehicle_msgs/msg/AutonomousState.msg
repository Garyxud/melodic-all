Header header

uint8 MISSION_NONE=1
uint8 MISSION_BRAKE_TEST=2
uint8 MISSION_ACCELERATION=3
uint8 MISSION_SKIDPAD=4
uint8 MISSION_TRACKDRIVE=5
uint8 MISSION_TEST=6

# mission to test correct heading of the vehicle while jacked up
# the front wheels should align between the next cones seen by the vehicle
uint8 MISSION_JACKED_UP_STEERING=100

uint8 STATE_OFF=1
uint8 STATE_READY=2
uint8 STATE_ACTIVE=3
uint8 STATE_EBS=4
uint8 STATE_FINISHED=5

uint8 state
uint8 mission
uint8[] failed_systems
