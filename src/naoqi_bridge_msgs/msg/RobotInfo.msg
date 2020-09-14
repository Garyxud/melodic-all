# info here is inspired by http://doc.aldebaran.com/2-1/naoqi/motion/tools-general-api.html?highlight=getrobotconfig#ALMotionProxy::getRobotConfig

# enums describing the robot type
uint8 NAO=0
uint8 ROMEO=1
uint8 PEPPER=2

# one of NAO, ROMEO, PEPPER
uint8 type

# "Model Type"   : "naoH25", "naoH21", "naoT14" or "naoT2".
string model
# "Head Version" : "VERSION_32" or "VERSION_33" or "VERSION_40".
string head_version
# "Body Version" : "VERSION_32" or "VERSION_33" or "VERSION_40".
string body_version
# "Arm Version"  : "VERSION_32" or "VERSION_33" or "VERSION_40".
string arm_version

# "Laser"        : True or False.
bool has_laser
# "Extended Arms": True or False.
bool has_extended_arms

# Number of Legs : 0 or 2
int32 number_of_legs
# Number of Arms : 0 or 2
int32 number_of_arms
# Number of Hands: 0 or 2
int32 number_of_hands
