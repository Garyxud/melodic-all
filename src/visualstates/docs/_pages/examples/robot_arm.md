---
title: Robotic Arm
permalink: /examples/robot_arm

sidebar:
  nav: "docs_examples"
---

The following example focuses on the hierarchies of automatas. The basic idea is to use predeveloped states to develop more complex behaviours. The example contains the following states -
* DrawI (Simple State with Letter I)
* DrawLove (Complex State with Letters L,O,V,E)
* DrawJdeRobot (Complex State with Letters J,D,E,R,O,B,O,T)
Hence we need low level states with letters from A to Z to generate any behavior which contains letters. The low level states contains the Robotic Arm code to drive the ARM to write letters. For example, State drawA would contain robot code to draw letter A.
Hence using hierarchies of these states we can develop complex states, i.e complex words/sentences. With a library of all 26 letters, we can Import the states(words) and construct any word/sentences.

## Steps to run the example
### Dependencies
* OpenCV
* VisualStates
* We assume you have created a ROS Workspace, cloned the latest release of VisualStates, compiled and sourced the workspace.

### ROS Package Generation
1. Copy the robot_arm folder as a ROS package to your workspace. In our case we name it 'catkin_ws'
```bash
cd catkin_ws
cd src
cp -r <path_to_visualstates_examples>/robot_arm/* .
cd ..
```
2. Generate the ROS Package of the behavior using the **visualstates**.
```bash
rosrun visualstates main.py <path_to_ros_workspace>/src/robot_arm/robot_arm.xml
```
3. Generate ROS package using `Actions -> Generate Python` menu.
4. Recompile the ROS Workspace and you would find a new package robot_arm in the package list. After compiling, source the workspace again.
```bash
catkin_make
source devel/setup.bash
```
5. Start roscore and run the generated ROS Node with Visualization Flag as True.
```bash
roscore
rosrun robot_arm robot_arm.py --displaygui=true
```
