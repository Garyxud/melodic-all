#################################################################
## Progress/State of a trajectory controller
#################################################################


Header header                  # header, ...
int32  state                   # controller state
uint32 progress                # passed waypoints 
uint32 progress_in_relation_to # id / seq nummer in oder to put the progess in realtion to a waypoint msg or spline msg
string info                    # details on the controller state in human form for debugging only 



int32 STATE_IDLE = 0                 # controller is inactive
int32 STATE_DRIVING = 1              # controller is driving and following a path
int32 STATE_FINISHED = 2             # controller finished a path
