Header header

bool present
# Boolean flag indicating that DBW system is present and communicating
# properly.  If this is false, none of the remaining data in the
# feedback can be considered valid.

# Boolean flag indicating that DBW system is in robotic mode.
bool robotic_mode

float32 steering_command
float32 steering_measure
# The current steering command and measured value of the steering
# axies in the range [0.0 to 1.0].  Typically 0.0 is full left and 1.0
# is full right, but the steering calibration can handle either
# direction.

float32 throttle_command
float32 throttle_measure
# The current throttle command and measured value of the throttle
# control in the range [0.0 to 1.0]

float32 brake_command
float32 brake_measure
# The current throttle command and measured value of the throttle
# control in the range [0.0 to 1.0]

bool estop_command
bool estop_measure

bool error_steering
bool error_throttle
bool error_brake
bool error_other
