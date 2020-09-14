# Name of the battery
string name

# Charge level of battery as percentage of maximum charge
float32 charge_level

# If true, the battery is being charged
bool is_charging

# When charging, this is the remaining time until fully charged.
# When discharging, this is the time until battery is empty.
# Non-zero values are considered valid.
duration remaining_time

# Total capacity of battery
float32 total_capacity

# Current capacity of battery
float32 current_capacity

# Voltage of battery
float32 battery_voltage

# Voltage of the supply breaker
float32 supply_voltage

# Voltage of the charger
float32 charger_voltage

