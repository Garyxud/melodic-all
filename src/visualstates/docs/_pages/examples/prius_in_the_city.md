---
title: Prius in the City
permalink: /examples/prius_in_the_city

sidebar:
  nav: "docs_examples"
---

The following example in VisualStates demonstrates the Prius Toyota Car wandering in the city. The environment is developed and simulated in Gazebo 9 and ROS Kinetic. The city is ambiguous to uncertanities like a human crossing a road, a crossing with turns, a car which needs to be overtaken, stop signs, speed signs on the road. The developed behavior in VisualStates is adaptable to all these uncertanities.

<iframe width="560" height="315" src="https://www.youtube.com/embed/1iYlJLJkESU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Steps to run the example
### Dependencies
We assume that you already installed ROS Kinetic and Gazebo 9 on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
We assume you have created a ROS Workspace, cloned the latest release of VisualStates, compiled and sourced the workspace.

### ROS Package Generation
Copy Prius messages, world, description packages from [Prius](/prius) and paste it in your ROS Workshop. Also copy the pre-developed VisualStates XML File. In our case lets name it catkin_ws.
```bash
cd catkin_ws
cd src
cp -r <path_to_visualstates_examples>/prius/* .
cp -r <path_to_visualstates_examples>/prius_simulate/prius_simulate.xml .
cp -r <path_to_visualstates_examples>/prius_in_city/* .
cd ..
```

Generate the ROS Package of the behavior using the **visualstates**.
```bash
rosrun visualstates main.py <path_to_ros_workspace>/src/prius_simulate/prius_simulate.xml
rosrun visualstates main.py <path_to_ros_workspace>/src/prius_overtake/prius_in_city.xml
```

Generate ROS package using `Actions -> Generate Python` menu. Recompile the ROS Workspace and you would find a new package prius_spawn in the package list. After compiling source the workspace again.

```bash
source devel/setup.bash
```

Add the CloverLeaf model to Gazebo models by adding the following line to your environment variables.
```bash
export GAZEBO_MODEL_PATH=<path_to_example>/prius_gazebo/models:$GAZEBO_MODEL_PATH
```

Start the Gazebo simulator and spawn Toyota Prius Car with Prius world, using the toyota prius world launch file
```bash
roslaunch prius_gazebo prius_city.launch
```

Run our generated ROS Node and visualize Toyota Prius Car Spawn Behavior.
```bash
rosrun prius_simulate prius_simulate.py --displaygui=true
rosrun prius_overtake prius_in_city.py --displaygui=true
```
