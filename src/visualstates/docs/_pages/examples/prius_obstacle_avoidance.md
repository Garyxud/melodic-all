---
title: Prius Obstacle Avoidance
permalink: /examples/prius_obstacle_avoidance

sidebar:
  nav: "docs_examples"
---

The following example in VisualStates demonstrates the Prius Toyota Car Obstacle Avoidance behavior in Gazebo 8 developed through ROS Kinetic. The following examples illustrates the Import Functionality present in VisualStates. The prebuilt behavior of ObstacleAvoidance for TurtleBot is imported and updated to be used for Prius Obstacle Avoidance behavior. Also, the child-states developed in PriusSpawn behavior are used to drive the car.

## Steps to run the example
### Dependencies
We assume that you already installed ROS Kinetic and Gazebo 8 on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
We assume you have created a ROS Workspace, cloned the latest release of VisualStates, compiled and sourced the workspace.

### ROS Package Generation
Copy Prius messages, world, description packages from [Prius](/prius) and paste it in your ROS Workshop. Also copy the pre-developed VisualStates XML File. In our case lets name it catkin_ws.
```bash
cd catkin_ws
cd src
cp -r <path_to_visualstates_examples>/prius/* .
cp -r <path_to_visualstates_examples>/priusObstacleAvoidance/* .
cd ..
```

Compile the package and source it.
```bash
catkin_make
source devel/setup.bash
```

Generate the ROS Package of the behavior using the **visualstates**.
```bash
rosrun visualstates main.py <path_to_ros_workspace>/src/prius_obstacle_avoidance/prius_obstacle_avoidance.xml
```

Generate ROS package using `Actions -> Generate Python` menu.

Recompile the ROS Workspace and you would find a new package prius_obstacle_avoidance in the package list. After compiling source the workspace again.

```bash
source devel/setup.bash
```

Add the CloverLeaf model to Gazebo models by adding the following line to your environment variables.
```bash
export GAZEBO_MODEL_PATH=<path_to_PriusSpawnExample>/prius_gazebo/models:$GAZEBO_MODEL_PATH
```

Start the Gazebo simulator and spawn Toyota Prius Car with Prius world, using the toyota prius world launch file
```bash
roslaunch prius_gazebo prius_obstacle_avoid.launch
```

Run our generated ROS Node and visualize Toyota Prius Car Obstacle Avoidance Behavior.
```bash
rosrun prius_obstacle_avoidance prius_obstacle_avoidance.py --displaygui=true
```



