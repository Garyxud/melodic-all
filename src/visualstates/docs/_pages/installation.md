---
layout: splash
title: Installation
permalink: /installation
---

## Installation from Ubuntu ROS package
If you have already installed ROS on your system, you can install visualstates with the following command.
```
sudo apt install ros-kinetic-visualstates
```
To run the visualstates run the following command
```
rosrun visualstates main.py
```

## Installation from Source

### Install Dependencies
```
sudo apt install ros-kinetic-desktop
sudo apt install python-pyqt5
sudo apt install python-pyqt5.qsci
```

### Install VisualStates in catkin_ws
The VisualStates tool is distributed as a ROS package. You can directly clone this repository in an active catkin workspace. After copying the repository as ROS package. You can run visualstates following these steps.
```
catkin_make
rosrun visualstates main.py
```

To run an example behavior please check the examples repository here: [VisualStates-examples](https://github.com/JdeRobot/VisualStates-examples)

