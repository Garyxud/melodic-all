---
title: Kobuki Obstacle Avoidance
permalink: /examples/kobuki-obstacle-avoidance

sidebar:
  nav: "docs_examples"
---

This behaviour demonstrates the Obstacle Avoidance behavior developed for Kobuki-Turtlebot robot developed using ROS Kinetic and simulated in Gazebo9. The behavior consists of 2 states -> `Move` and `Avoid` and transitions based on the laser sensor data to detect and avoid obstacles like wall. Here's a small demo video of the behaviour:

<iframe width="560" height="315" src="https://www.youtube.com/embed/o-SAe_qwOMc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" align="middle" allowfullscreen></iframe>

## Steps to run the example
#### Dependencies
We assume that you already installed ROS and Gazebo on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

We also need worlds from JdeRobot repositories. To be able to install jderobot gazebo assets. Please follow the steps here: https://jderobot.org/Installation
We add required commands here for completeness
```bash
sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list
# for ubuntu 16.04 LTS (64 bit)
deb [arch=amd64] http://jderobot.org/apt xenial main
EOF'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4
sudo apt update
sudo apt install jderobot-gazebo-assets
```
To be able to complete the JdeRobot Gazebo assets installation we need to source the JdeRobot installation script
```bash
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```
***This is important that you need to run your gazebo simulation on the console that you have sourced your JdeRobot installation otherwise Gazebo simulator cannot find models and worlds of JdeRobot***

#### ROS package generation
First we must generate the ros package of the behavior using the **visualstates**.
```bash
rosrun visualstates main.py <path_to_visualstates_examples>/kobuki_obstacle_avoidance/kobuki_obstacle_avoidance.xml
```
Using `File -> Save As` save the behavior in an empty directory that is also in an active `catkin_workspace`. Since code generation will create required files to make the directory a ROS package, you should have different directory for every new behavior. Now, we can generate ROS package using `Actions -> Generate Python` menu.

Navigate to your `catkin_workspace` and run `catkin_make`. As an output of `catkin_make` you will see the `kobuki_obstacle_avoidance` listed as a ROS package.
```bash
cd catkin_ws
catkin_make
```
Start the roscore
```bash
roscore
```
Start the Gazebo simulator, using the gazebo_ros package
```bash
rosrun gazebo_ros gazebo kobuki-simple-ros.world
```
Run our generated package
```bash
rosrun kobuki_obstacle_avoidance kobuki_obstacle_avoidance.py --displaygui=true
```

