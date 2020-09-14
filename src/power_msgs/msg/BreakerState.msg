#
# Gives feedback on a Breaker
# Similar to REP-117, missing measurements should be denoted with a NAN
#

uint8 STATE_DISABLED = 0
uint8 STATE_ENABLED = 1
uint8 STATE_ERROR = 2

# Name of the breaker
string name

# The state of the breaker, should be one of the values seen above.
uint8 state

# The current flowing through the breaker, in Amps.
float32 current

# The temperature of the breaker, in degrees C.
float32 temperature

# The rated current limits of the breaker.
# Outside of these limits, breaker should trip.
# If breaker cannot measure or protect against negative
# current, min_rated_current should be 0.0
float32 min_rated_current
float32 max_rated_current

# The rated temperature limit of breaker.
# Above this temperature, breaker should trip.
float32 max_rated_temperature
