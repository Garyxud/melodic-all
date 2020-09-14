# odom_frame_publisher


![Developed By OUXT Polaris](img/logo.png "Logo")

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/odom_frame_publisher.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/odom_frame_publisher)|[![Build Status](https://travis-ci.org/OUXT-Polaris/odom_frame_publisher.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/odom_frame_publisher)|

## nodes
### odom_frame_publisher_node

This node subscribe current Pose and Twist topic and publish odometry frame based on [REP105](https://www.ros.org/reps/rep-0105.html).

Params
| *param* | *Type* | *Description* | *Dfault* |
|---------|--------|--------|--------|
|current_pose_topic|string|Topic name of the current pose|/current_pose|
|current_twist_topic|string|Topic name of the current twist|/current_twist|
|robot_frame_id|string|frame_id of the robot|base_link|
|map_frame_id|string|frame_id of the map|map|
|odom_frame_id|string|frame_id of the odom|odom|

Topics
| *Topic* | *Type* | *Pub/Sub* | *Description* |
|---------|--------|--------|--------|
|param (current_pose_topic)|geometry_msgs/PoseStamped|Sub|Current Pose in map frame|
|param (current_twist_topic)|geometry_msgs/TwistStamped|Sub|Current Twist in robot frame|
|tf|tf/tfMessage|Pub,Sub|Publishing odom frame based on [REP105](https://www.ros.org/reps/rep-0105.html)|
|~odom_pose|geometry_msgs/PoseStamped|Pub|Pose in the odom frame|