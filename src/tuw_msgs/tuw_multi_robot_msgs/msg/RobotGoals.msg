#################################################################
## used to describe a set of destinations for a robot
## robot_name is used as id and often used as namespace for topics
#################################################################

string robot_name                   # the name of the robot
geometry_msgs/Pose[] destinations   # Describes a set of destinations, where the last one is the final goal of the trip
                                    # if there are more than one destinations the first one is the start pose
                                    # else the current pose of the robot is used as start
 
 

