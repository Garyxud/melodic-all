# TODO add motor state/error definitions

string name

uint8 state
uint8 error

float32 velocity
float32 effort
float32 position

float32 temperature

# TODO: velocity/position gains

float32 motor_ratio
float32 motor_angle_offset
float32 motor_angle_offset_estimated
float32 joint_ratio
