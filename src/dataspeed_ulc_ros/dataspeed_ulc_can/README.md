# dataspeed_ulc_can
 
## ulc_node
This node allows ROS users to command the Universal Lat/Lon Controller (ULC) using ROS messages instead of having to manually construct and populate CAN messages. The node subscribes to three different command topics that alter the behavior of the node. It is recommended to make sure that only one command topic is being used at a given time.

### Subscribed Topics
#### ulc_cmd ([dataspeed_ulc_msgs/UlcCmd](../dataspeed_ulc_msgs/msg/UlcCmd.msg))

Input command for the ULC. In addition to the speed and steering command inputs, this topic also configures the behavior of the ULC. It allows the user to turn the speed and steering components of the ULC on and off, switch shifting and steering modes, and configure longitudinal and lateral acceleration limits.

#### cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

Simplified interface to the ULC. When messages on this topic are used to command the ULC:

  - The speed component of the ULC is activated and tracks the `linear.x` field of the `geometry_msgs/Twist` message, assuming the units are m/s
  - The steering component of the ULC is activated in yaw rate mode and tracks the `angular.z` field of the `geometry_msgs/Twist` message, assuming the units are rad/s
  - All longitudinal and lateral acceleration limits use the default settings outlined in the ULC User's Guide

#### cmd_vel_stamped ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))

The header of the incoming `geometry_msgs/TwistStamped` messages are ignored, and instead treated identically to the `cmd_vel` topic.

#### can_rx ([can_msgs/Frame](http://docs.ros.org/melodic/api/can_msgs/html/msg/Frame.html))

This topic listens to the CAN traffic on the Dataspeed ADAS Kit CAN bus and uses the data to publish the messages on the `ulc_report` topic.

### Published Topics

#### can_tx ([can_msgs/Frame](http://docs.ros.org/melodic/api/can_msgs/html/msg/Frame.html))

This topic transmits ULC command and configuration CAN messages to the Dataspeed ADAS Kit CAN bus to control the ULC running in the ADAS Kit firmware. These CAN messages are constructed based on how the user is publishing command messages on the input topics.

#### ulc_report([dataspeed_ulc_msgs/UlcReport](../dataspeed_ulc_msgs/msg/UlcReport.msg))

Feedback data from the ULC running in the ADAS Kit firmware.