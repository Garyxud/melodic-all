# ypspur_ros node

The topic names will be migrated to ROS recommended namespace model.
Set `compatible` parameter to `1` to use new topic names.

## Subscribed topics

### ~/cmd_vel (new: cmd_vel) [geometry_msgs/Twist]

Vehicle velocity control command input.

### ~/cmd_joint (new: joint_trajectory) [trajectory_msgs/JointTrajectory]

Joint trajectory control command input.

### ~/control_mode (new: control_mode) [ypspur_ros/ControlMode]

Motor control mode input.
(velocity control, torque control, passive (short circuit) brake, and open circuit mode)

### ~/joint_position (new: joint_position) [ypspur_ros/JointPositionControl]

Joint position control command without time constraint. (Time optimal control.)

### ~/dio? (new: dio?) [ypspur_ros/DigitalOutput]

Digital IO output command.
This topic name is configurable through `dio?_name` parameter.

## Publihed topics

### ~/odom (new: odom) [nav_msgs/Odometry]

Odometry pose.

### ~/joint (new: joint_states) [sensor_msgs/JointState]

Joint status array.

### ~/wrench (new: wrench) [geometry_msgs/WrenchStamped]

Estimated vehicle force and torque.

### ~/digital_input (new: digital_input) [ypspur_ros/DigitalInput]

Digital IO status.

### ~/ad/name (new: ad/name) [std_msgs/Float32]

Analog input status.
This topic name is configurable through `ad?_name` parameter.

## Parameters

### ~/port [string: "/dev/ttyACM0"]

### ~/ypspur_bin [string: "/usr/local/bin/ypspur-coordinator"]

### ~/param_file [string: ""]

Path to your vehicle parameter file for yp-spur.

### ~/cmd_vel_expire [double: -1.0]

Expire duration of cmd_vel. After this duration since receiving cmd_vel, velocity and angular velocity is set to zero.
Negative or zero value disables this feature.

### ~/ad?_enable [bool: false]

### ~/ad?_name [string: "ad" + i]

Published topic name

### ~/ad?_gain [float: 1.0]

### ~/ad?_offset [float: 0.0]

### ~/dio?_enable [bool: false]

### ~/dio?_name [string: "dio" + i]

Subscribed topic name and published IO pin name.

### ~/dio?_output [bool: true]

### ~/dio?_input [bool: false]

### ~/dio?_default [string: "HIGH_IMPEDANCE"]

Default output state.

### ~/odom_id [string: "odom"]

Odometry frame id.

### ~/base_link_id [string: "base_link"]

base_link frame id.

### ~/hz [float: 200.0]

Frequency to publish odometry data.

### ~/odometry_mode [string: "diff"]

### ~/max_joint_id [int: 32]

### ~/joint?_enable [bool: false]

### ~/joint?_name [string: "joint" + i]

### ~/vel [float: max velocity specified in the parameter file]

### ~/acc [float: max acceleration specified in the parameter file]

### ~/angvel [float: max angular velocity specified in the parameter file]

### ~/angacc [float: max angular acceleration specified in the parameter file]



