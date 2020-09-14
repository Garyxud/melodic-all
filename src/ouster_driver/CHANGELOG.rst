^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ouster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2019-03-28)
------------------
* Sensor model and firmware version validation
* Added sensor model and firmware version validation. The driver currently only works for OS-1-64 and on firmware version 1.10 some configuration parameters disappeared.
* Parameter validation and new pointcloud types
* Added parameter validation to avoid reinitializing the LiDAR. Added new pointcloud types XYZIRF and XYZIRFN. Changed pulse_mode parameter from integer to string for readability. Updated the readme file.
* Contributors: Alexander Carballo, alexandrx

0.1.6 (2019-03-13)
------------------
* Renamed ouster_ros to ouster_driver

0.1.5 (2019-03-13)
------------------
* updated package.xml

0.1.4 (2019-03-13)
------------------
* Updated project name to follow ros standards

0.1.3 (2019-03-06)
------------------
* Updated LICENSE

0.1.2 (2019-03-06)
------------------
* Changed operation mode from integer to string to ease reading.
* New pointcloud mode format added
* Improved parameter documentation
* removed the old bool pointcloud_mode_xyzir parameter (to match Velodyne's mode) and replaced with a way to specify the desired pointcloud format, several options added including Ouster's native pointcloud format.
* Added install commands
* run CI on kinetic and melodic only
* Fixed launch file to deal with hidden parameters, fixed readme.md to update the TODO, fixed os1_node.cpp to reduce by half the scan duration variable in case of 20Hz operation.
* Updated the driver to support advanced parameter configuration (lidar-mode, pulse-mode and window-rejection-enable)
* Eliminated the ouster-ros static library 
* Updated readme.md incorrect info for lidar address configuration.
* Switched to industrial-ci
* First commit for Autoware version branch. Modified the original Ouster ROS code to suit Autoware needs. Code structure is more simple, only ROS driver code is kept, and the readme.md file has more details on how to configure the sensor and use this driver. To further ease Autoware integration, Velodyne compatibility options were added and tested with nd-_matching, ray-ground-filter and euclidean-cluster nodes.
* Improve positional argument parsing in viz
* Use allocate-shared for Eigen compatibility
* Support building visualizer with newer VTK
* Readme tweaks and new example data
* Update readme.md
* updated the readme to include viz in the contents
* Initial commit of the example visualizer
* Uses VTK6 to display point clouds and range/intensity/noise images
* No dependencies other than VTK6, Eigen3, and a C++11 compiler
* Currently only supports linux; tested on multiple distributions and platforms
* Use correct units in sensor-msgs::Imu
* Deal with time going backwards in replays
* Contributors: Alexander Carballo, Dima Garbuzov, Roy Rinberg, alexandrx, amc-nu, hooic, rafficm, rrinberg
