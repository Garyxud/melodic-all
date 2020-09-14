std_msgs/Header header
bool pedals_enabled
bool steering_enabled
bool speed_preempted
bool steering_preempted
bool override_latched
bool timeout

uint8 YAW_RATE_MODE=0
uint8 CURVATURE_MODE=1
uint8 steering_mode

uint8 LOOSE_TRACKING_MODE=0
uint8 TIGHT_TRACKING_MODE=1
uint8 tracking_mode

float32 speed_ref
float32 speed_meas
float32 accel_ref
float32 accel_meas
float32 max_steering_angle
float32 max_steering_vel
