uint8 state
uint8 error
# Charging mode is defined as:
#  0 - float charge (last 10%)
#  1 - charging
#  2 - not charging
uint8 charging_mode
uint8 limit_cause
uint8 balancing_mode

float32 battery_voltage
float32 battery_half_voltage
float32 charger_voltage
float32 supply_voltage

float32 phase1_current
float32 phase2_current

float32 charger_temp
float32 battery_temp
float32 supply_connector_temp

float32 fan_speed

float32 battery_capacity
float32 battery_energy
