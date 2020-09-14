# ypspur_ros

## Package summary

This package provides a ROS wrapper node for [YP-Spur](https://github.com/openspur/yp-spur) vehicle control backend. 
The wrapper node supports almost all features provided by YP-Spur including vehicle control, digital IO, A/D input, multi-DOF joint control, etc.

This package requires the latest version of [YP-Spur](https://github.com/openspur/yp-spur).

## Nodes
### [ypspur_ros](doc/ypspur_ros.md) 
Wrapper node.

### joint_tf_publisher
sensor_msgs/JointState message to tf converter.

### joint_position_to_joint_trajectory

ypspur_ros/JointPositionControl to trajectory_msgs/JointTrajectory converter.

## License

ypspur_ros is available under BSD license.
