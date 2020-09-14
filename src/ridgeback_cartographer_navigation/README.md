# Ridgeback Cartographer Demo

This tutorial shows you how to use [move_base](http://wiki.ros.org/move_base) with [Google Cartographer](https://github.com/googlecartographer) to perform autonomous planning and movement with simultaneous localization and mapping (SLAM), on a simulated Ridgeback, or a factory-standard Ridgeback with two laser scanners publishing on the */front/scan* and */rear/scan* topics.

Developed by Google, Cartographer is an open-source library used for real-time simultaneous localization and mapping (SLAM). Using LIDAR data, it generates submaps which are later optimized and scan-matched to provide real-time loop closure. As a result, Cartographer is able to successfully close loops and generate a consistent map each time whereas Gmapping can occasionally fail to do so, resulting in unique maps generated after every iteration [[1](https://lup.lub.lu.se/student-papers/search/publication/8915402)]. A variety of sensor configurations are supported allowing Cartographer to be used for a broad range of applications. In this demo, Cartographer uses odometry and data from two laser scanners to perform real-time SLAM with the Ridgeback.

To adapt this demo to your own Ridgeback, you may need to clone the [ridgeback_cartographer_navigation](http://github.com/ridgeback/ridgeback_cartographer_navigation.git) repository, and modify the relevant parameters. To learn about move_base and the navigation stack, see the [Navigation Tutorials](http://wiki.ros.org/navigation/Tutorials). To learn more about Google Cartographer for ROS, see the [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/) documentation.

## Instructions

  1. To get started with 2-D SLAM using Google Cartographer, clone this repository into your working directory:

     `git clone http://github.com/ridgeback/ridgeback_cartographer_navigation.git`

  2. Install the following ROS packages:

     `rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y`

  3. Build the workspace and open two new terminal/tabs, source the workspace for each terminal/tab:

     `source devel/setup.bash`

      - Launch the Gazebo simulation with the *dual_sick_lasers* config:

        `roslaunch ridgeback_gazebo ridgeback_world.launch config:=dual_sick_lasers`

      - Launch the Cartographer node to begin SLAM:

        `roslaunch ridgeback_cartographer_navigation cartographer_demo.launch`

      - Launch Rviz:
        `roslaunch husky_viz view_robot.launch`

  4. In the Rviz visualizer, make sure the visualizers in the Navigation group are enabled.

  5. Use the 2D Nav Goal tool in the top toolbar to select a movement goal in the visualizer. Make sure to select an unoccupied (dark grey) or unexplored (light grey) location.

  6. As the robot moves, you should see the grey static map (map topic) grow. There might be discrete jumps in the map as the Cartographer algorithm attempts to localize the robot.

  ![Ridgeback World Map](ridgeback_cartographer.png)

  7. To save the generated map, you can run the map_saver utility:

     `rosrun map_server map_saver -f <filename>`

#### Tuning Cartographer

To tune Cartographer for low latency SLAM, edit the *ridgeback.lua* configuration file found in the *ridgeback_cartographer_navigation/config* directory.

For more information on tuning, click [here](http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)

[1] T. Coroiu and O. Hinton, "A Platform for Indoor Localisation,
Mapping, and Data Collection using an
Autonomous Vehicle," M.S. thesis, Lund Univ., Lund, Sweden, 2017.
