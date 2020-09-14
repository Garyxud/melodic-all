^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh_geo_fog_3d_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2020-01-21)
-----------
* No changes to driver

1.3.2 (2020-01-17)
-----------
* Fixing build bug where driver would fail to build if CATKIN_ENABLE_TESTING was not set to true, due to an errant target_link_libraries line.
* Contributors: LaCelle, Zachary

1.3.1 (2020-01-14)
-----------
* Adding odometer pulsePerMeter as a parameter.
* Adding the X position calculated from odometery movement to the nav_msgs/Odometry wheel encoder message
* Contributors: Bostic, Trevor R, LaCelle, Zachary

1.3.0 (2019-12-2)
-----------
* Fixing bug where we were reporting lat/lon for NavSatFix messages in radians, when ROS asks for them in degrees
* Fixing missing include
* Fixing missing find_package
* Updating packaging and cmakes to conform to catkin_lint
* Rewrote bounding functions
* Fixing up some documentation issues that rosdoc_lite found.
* Fixing spacing to match 2-space indents everywhere. Also fixing a missing break within a switch/case.
* Adding the GPS transform to the static broadcaster
* Cleaning up changelog, and adding MITRE's open soure usage email request header to files. Also adding license information to mainpage.dox
* Added ROS API documentation
* Adding a rosdoc yaml, and removing the old Doxyfile
* Removing the yet-unimplemented nodelet classes
* Adding copyright file headers.
* Moving license files to be under each package, since theoretically they could be licensed separately.
* Adding license information.
* Updating cmakelists to build roslint targets correctly
* Adding devops scripts
* Contributors: Bostic, Trevor R, LaCelle, Zachary

1.2.0 (2019-09-27)
-----------
* Filter Options and Magnetometer
* Defaulting to turning off velocity heading, since KVH recommended it for the Dual.
* Implemented filter options packet configuration.
* Added navsatfix and imu messages for the raw data to make graphing easier.
* Fixed some problems with getting raw packets.
* Added raw gnss and sensor messages to those output by the driver.
* Fixed launch file syntax and printing error.
* Added ability to customize initialization options.
* Set wheel encoder frame_id to base_link.
* Updated baud setter to have different values for each of the ports.
* Added odom state message and message publishers to kvh driver node.
* Added odometer state packet to the kvh driver. Next will be implementing messages.
* Added utm tests.
* Fixing packet storage utm problem.
* Fixed utm struct problem.
* Fixes for cpp check warnings.
* Moving and fixing the release script.
* Contributors: Bostic, Trevor R, LaCelle, Zachary

1.1.0 (2019-08-13)
-----------
* Moving msgs into their own package
* Updating package.xml and CMakeLists.txt to handle the new packaging scheme
* Fixes of tfs for orientation
* Various fixes to covariances, especially when we temporarily lose communications
* Updating the IMU data publishing to match ROS schemes
* Fixing a bug in the TF for GPS
* Many frame ID changes
* Variable baud rates
* Adding an autobaud node
* Bug fixes in the UTM and Fix packets
* More ublox custom messages

1.0.0 (2019-04-25)
-----------
* Initial release of the ROS KVH GEO FOG 3D driver package, reading basic state and information packets.
