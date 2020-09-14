# Overview
This ROS package provides teleoperation using multi-key input for turtlesim/turtlebot3
You can move turtlebot more easily (like game character!)

[![turtle_teleop_multi_key movement](https://img.youtube.com/vi/bDvn6tUCVmk/0.jpg)](https://youtu.be/bDvn6tUCVmk)

# Requirements
python >=2.7
pynput >=1.6.8

# Tested at
## Remote PC
Ubuntu 18.04
ROS Melodic
python 2.7.17
pynput 1.6.8

## Turtlebot PC
Ubuntu Mate 18.04
ROS Melodic

# Launch
for turtlebot 3:
```
roslaunch turtle_teleop_multi_key turtle_teleop_multi_key.launch
```
for turtlesim:
```
roslaunch turtle_teleop_multi_key turtle_teleop_multi_key.launch topic_type:=turtlesim
```

# License
turtle_teleop is released with the MIT License. For full terms and conditions, see the [LICENSE file](./LICENSE.txt)
