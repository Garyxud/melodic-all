# control the way point/trajectory navigation
int8 control

int8 STOP  = 0
int8 START = 1
int8 PAUSE = 2

# name of the way point(s) / trajectory to be execute
# leave empty, when stopping or pausing
string goal_name