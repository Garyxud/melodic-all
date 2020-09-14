#Message for sensing/control of an indepenend N wheel steering platform 
Header header

#the type defines the usage and units such as cmd_torque, cmd_acceleration, cmd_velocity, cmd_position, measured_torque, measured_acceleration, measured_velocity, measured_position
string type_steering
string type_revolute 

#actual sensing/control variables of the defined type
float64[] steering
float64[] revolute