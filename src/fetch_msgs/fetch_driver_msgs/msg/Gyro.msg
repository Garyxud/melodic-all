Header header

uint32 flags

# Scaled and offset gyro axis values
geometry_msgs/Vector3 data

# Raw data from gyro (unscaled, and unofffset) 
geometry_msgs/Vector3 raw

# Scale values used for each axis
geometry_msgs/Vector3 scale

# Offset value used for each axis
geometry_msgs/Vector3 offset


# Glitch was detected
uint32 GLITCH = 1

# Problem initializing gyro
uint32 INIT_ERROR = 2

# Error reading data
uint32 READ_ERROR = 4

# Interrupt for new data is not being triggered fast enough
uint32 INTERRUPT_TOO_SLOW = 8

# Interrupt for new data is being triggrered to quickly
uint32 INTERRUPT_TOO_FAST = 16

# No sensor is availble
uint32 SENSOR_NOT_PRESENT = 32

# Robot is definitely moving, gyro output should be non-zero 
uint32 DEFINITELY_MOVING = 536870912   # 1<<29

# Robot is definitely stopped, gyro offset can be adjusted
uint32 DEFINITELY_STOPPED = 1073741824   # 1<<30

# Robot gyro has any type of error, can be used for ignoring
# invalid gyro data even without understanding any specific
# error types
uint32 ANY_ERROR = 2147483648   # 1<<31

