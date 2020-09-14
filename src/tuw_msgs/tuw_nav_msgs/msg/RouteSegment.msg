
uint32 id                     # Segment id
uint8 type                    # Segment type
                              # NA      = 0
                              # LINE    = 1
                              # ARC     = 2
                              # SPIROS  = 3
                              # SPLINE  = 4
uint8 orientation             # Segment orientation
                              # CLOCKWISE  = 0
                              # COUNTER_CLOCKWISE  = 1
uint8 motion_type             # Motion type
                              # NA = 0
                              # FLAT = 1
                              # DWA_SLOW = 2
                              # DWA_FAST = 3
                              # MPC_SLOW = 4
                              # MPC_FAST = 5
                              # PI_SLOW = 6
                              # PI_FAST = 7
geometry_msgs/Pose start
geometry_msgs/Pose end
geometry_msgs/Pose center    
int8 level

