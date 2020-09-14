#################################################################
## Presents dynamic parameters of a robot
#################################################################

Header header                            # the creation time
string robot_name                        # the name of the robot (used in preconditions and topics)
geometry_msgs/PoseWithCovariance pose    # the robots current pose within the frame related to the msgs header
int32 shape                              # the shape of the robots (see enums)
float32[] shape_variables                # shape variables to define width height, ...
RoutePrecondition sync                   # the current position in the last received plan (-1 means none)
int32   mode                             # the mode of operation
int32   status                           # the status of the robot
int32   good_id                          # the good id attached to the robot
int32   order_id                         # the order id scheduled to this robot (-1: none)
int32   order_status                     # the status of the assigned order

# mode
int32 MODE_NA = 0                   # undefined mode
int32 MODE_IDLE = 1                 # robot is idle
int32 MODE_SEGMENT_FOLLOWING = 2    # robot is in mode segment following
int32 MODE_PICKUP = 3               # robot is picking up goods

# status
int32 STATUS_DRIVING = 0            # robot is driving
int32 STATUS_STOPPED = 1            # robot has stopped
int32 STATUS_DONE = 2               # robot has finished its last job
int32 STATUS_BROKEN = 3             # robot is broken and not ready for any task

# good_id
int32 GOOD_EMPTY = -1               # no goods attached
int32 GOOD_NA = -2                  # undefined good

# shape
int32 SHAPE_CIRCLE = 0                 # robot is in shape of a circle    ShapeVars

# order_status
int32 ORDER_NONE = 0                # no order assigned
int32 ORDER_APPROACH = 1            # the robot approaches the first station of the order
int32 ORDER_PICKUP = 2              # the robot picks up a good at the station
int32 ORDER_TRANSPORT = 3           # the robot currently transports a good from one station to another
int32 ORDER_DROP = 4                # the robot drops a good at the last station of its order, finishing the order
