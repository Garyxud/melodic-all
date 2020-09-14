---
title: Prius Crossing
permalink: /examples/prius_crossing

sidebar:
  nav: "docs_examples"
---

The following example in VisualStates demonstrates the Prius Toyota Car behavior at a crossing in Gazebo 9 developed through ROS Kinetic. The following examples illustrates the several functionalities of the tool. The example starts with prius on-road, waits for a person to cross, detects speed-limit sign,  and then makes a left turn. The Prius car follows the road maintaining steer, break and throttle based on vision and control algorithm. The vision algorithm detects the lane in the camera using a pipeline of HSV color filteration followed by a P controller for steer and throttle control.

<iframe width="560" height="315" src="https://www.youtube.com/embed/Tqh14Q2boXY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Steps to run the example
### Dependencies
We assume that you already installed ROS Kinetic and Gazebo 9 on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

### ROS Package Generation
Copy Prius messages, world, description packages from [PriusData](/prius) and paste it in the ROS Workshop. Also clone the VisualStates package and copy the VisualStates prius_crossing.xml file which contains the crossing behavior. The example requires OpenCV installed.
```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/JdeRobot/VisualStates.git
cp -r <path_to_visualstates_examples>/prius/* .
cp -r <path_to_visualstates_examples>/prius_crossing/* .
cd ..
catkin_make
source devel/setup.bash
```

Generate the ROS Package of the behavior using the **visualstates**.
```bash
rosrun visualstates main.py <path_to_ros_workspace>/src/prius_crossing/prius_crossing.xml
```

Generate ROS package using `Actions -> Generate Python` menu. Recompile the ROS Workspace and you would find a new package prius_example in the package list. After compiling source the workspace again.
```bash
source devel/setup.bash
```
Add the Prius Crossing Environment model to Gazebo models by adding the following line to your environment variables.
```bash
export GAZEBO_MODEL_PATH=<path_to_PriusSpawnExample>/prius_gazebo/models:$GAZEBO_MODEL_PATH
```
Start the Gazebo simulator and spawn Toyota Prius Car with Prius world, using the toyota prius world launch file
```bash
roslaunch prius_gazebo prius_crossing.launch
```
Run our generated ROS Node and visualize Toyota Prius Car Example Behavior.
```bash
rosrun prius_crossing prius_crossing.py --displaygui=true
```
