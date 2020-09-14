^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ypspur_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2019-12-16)
------------------
* Fix communication error handling (`#64 <https://github.com/openspur/ypspur_ros/issues/64>`_)
* Drop support for yp-spur<1.17.0 to simplify the code (`#63 <https://github.com/openspur/ypspur_ros/issues/63>`_)
* Add sleep to ensure publishing rosout before exit (`#61 <https://github.com/openspur/ypspur_ros/issues/61>`_)
* Fix coding style (`#62 <https://github.com/openspur/ypspur_ros/issues/62>`_)
* Add post-release test script (`#58 <https://github.com/openspur/ypspur_ros/issues/58>`_)
* Disable CI build on indigo (`#59 <https://github.com/openspur/ypspur_ros/issues/59>`_)
* Contributors: Atsushi Watanabe

0.3.0 (2019-05-09)
------------------
* Add parameter to set expire duration of cmd_vel (`#55 <https://github.com/openspur/ypspur_ros/issues/55>`_)
* Ignore outdated JointTrajectory command (`#54 <https://github.com/openspur/ypspur_ros/issues/54>`_)
* Fix exception type (`#52 <https://github.com/openspur/ypspur_ros/issues/52>`_)
* Fix subprocess handling (`#48 <https://github.com/openspur/ypspur_ros/issues/48>`_)
* Add device error status diagnostic output (`#46 <https://github.com/openspur/ypspur_ros/issues/46>`_)
* Fix test dependencies and update manifest (`#42 <https://github.com/openspur/ypspur_ros/issues/42>`_)
* Contributors: Atsushi Watanabe

0.2.0 (2018-06-07)
------------------
* Add CI build for melodic (`#37 <https://github.com/openspur/ypspur_ros/issues/37>`_)

  * Also rename ci script directory

* Add encrypted token for image caching (`#35 <https://github.com/openspur/ypspur_ros/issues/35>`_)
* Migrate to ROS recommended namespace model (`#31 <https://github.com/openspur/ypspur_ros/issues/31>`_)
* Fix --enable-get-digital-io arg to ypspur-coordinator (`#33 <https://github.com/openspur/ypspur_ros/issues/33>`_)
* Fix installation of nodes (`#30 <https://github.com/openspur/ypspur_ros/issues/30>`_)
* Fix variable and class naming styles (`#29 <https://github.com/openspur/ypspur_ros/issues/29>`_)
* Contributors: Atsushi Watanabe

0.1.0 (2018-04-19)
------------------
* Update CI settings (`#25 <https://github.com/openspur/ypspur_ros/issues/25>`_)

  * Use docker hub as a cache

* Fix build dependencies (`#24 <https://github.com/openspur/ypspur_ros/issues/24>`_)

  * Fix cmake target build deps
  * Fix package deps

* Fix joint state timestamp coherency (`#23 <https://github.com/openspur/ypspur_ros/issues/23>`_)

  * Fix joint state timestamp coherency
  * Use system time if yp-spur didn't provide stamp

* Add build test on indigo. (`#20 <https://github.com/openspur/ypspur_ros/issues/20>`_)
* Fix coding style. (`#19 <https://github.com/openspur/ypspur_ros/issues/19>`_)
* Fix timestamp in simulation mode. (`#18 <https://github.com/openspur/ypspur_ros/issues/18>`_)
* Add build test. (`#17 <https://github.com/openspur/ypspur_ros/issues/17>`_)

  * Add build test.
  * Fix indent in CMakeFile.
  * Fix package deps.

* Support running ypspur-coordinator by using PATH env. (`#14 <https://github.com/openspur/ypspur_ros/issues/14>`_)
* Use find_package(ypspur) instead of catkin_package. (`#12 <https://github.com/openspur/ypspur_ros/issues/12>`_)
* Use CMake version of ypspur. (`#10 <https://github.com/openspur/ypspur_ros/issues/10>`_)

  * Also, fix dummy dependency to system_lib.

* adds README (`#9 <https://github.com/openspur/ypspur_ros/issues/9>`_)
* publishes digital input port state (`#8 <https://github.com/openspur/ypspur_ros/issues/8>`_)
* fixes to compile with old versions of YP-Spur which does not have joint_ang_vel command
* adds error handling on joint trajectory control
* joint_position_to_joint_trajectory: temporary removes time to accelerate
* joint_position_to_joint_trajectory: skips duplicated joint command
* joint_position_to_joint_trajectory: takes care of the current joint position
* adds joint_position_to_joint_trajectory converter
* fixes uncleared joint trajectory command cache
* increases cmd_joint input buffer
* allows divided joint trajectory command
* adds joint trajectory control
* fixes DIO default status parameter setting
* supports joint effort output (`#4 <https://github.com/openspur/ypspur_ros/issues/4>`_)

  * This also fixes a bug that joint effort field was filled by velocity value on the version of YP-Spur without joint control support.
  
* changes default vel/acc settings to use values defined in the parameter file
* fixes ypspur-coordinator process monitoring
* adds vehicle control mode interface
* fixes digital IO control
* adds param to set tf timestamp offset
* adds simple simulation of robot control and joint angle control
* adds ros::shutdown before quiting the main loop
* fixes A/D output message type
* joint_tf_publisher: adds node to generate tf messages from joint topic
* adds combined joint position control input
* adds parameters to specify A/D port name in the output message
* adds digital I/O port output
* changes A/D related parameter names (ad_enable0 to ad0_enable)
* changes names of the joint control inputs according to the specified joint names
* supports more than two joint control
* adds retry and error handling in getID script
* improves ypspur-coordinator availability check
* ROS interface of the mobile robot control platform "YP-Spur"
* Contributors: Atsushi Watanabe
