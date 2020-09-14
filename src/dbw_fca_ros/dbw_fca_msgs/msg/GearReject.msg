uint8 value

uint8 NONE=0              # Not rejected
uint8 SHIFT_IN_PROGRESS=1 # Shift in progress
uint8 OVERRIDE=2          # Override on brake, throttle, or steering
uint8 VEHICLE=5           # Rejected by vehicle (try pressing the brakes)
uint8 UNSUPPORTED=6       # Unsupported gear command
uint8 FAULT=7             # System in fault state
