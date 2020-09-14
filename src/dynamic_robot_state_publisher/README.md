# dynamic_robot_state_publisher

Improved ROS `robot_state_publisher` which can update the robot model via dynamic_reconfigure.

## Nodes

### dynamic_robot_state_publisher

Use this node as a replacement for `robot_state_publisher/robot_state_publisher`.

It behaves the same way, except it also spawns a dynamic reconfigure server in the node's namespace.
This server can receive updates of the robot model, to which it reacts by reloading the robot model.

## Rationale

This package was created as one of the workarounds to issue [Reload robot model (e.g. during calibration)](https://github.com/ros/robot_state_publisher/issues/29).

If you read that issue, you learn that this problem is quite complex and this package is a solution just to a subclass of the problems.
E.g. 3rd party packages do not expect the `robot_description` parameter to change, and so you have to somehow tell them to do so (e.g. by restarting their nodes).

One of the things that this package does and is probably aginst good practices is
it tries to delete static transforms that disappeared after model reload. This
deletion is implemented by reconnecting the frames to virtual frame 
`DynamicRobotStatePublisher::DELETED_STATIC_TFS_FRAME`. This effectively breaks the
assumption that a static TF is valid all the time. These static TFs are not. You 
have to count with that in the rest of your codebase.

## Alternatives

* https://github.com/ros/robot_state_publisher/pull/31 (Adds a `TriggerService`) which you can call to reload the model from param server.
* [mutable_robot_state_publisher](https://github.com/RethinkRobotics/mutable_robot_state_publisher.git): A more complex thing that allows you to send URDF fragments to the state publisher.