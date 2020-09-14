^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataspeed_pds_lcm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-07-28)
------------------
* Add missing rostest dependency
* Contributors: Kevin Hallenbeck

1.0.4 (2020-07-24)
------------------
* Don't extract the LCM binaries for the buildfarm since they're available for modern distributions
* Change liblcm dependency to match official rosdep rule
  https://github.com/ros/rosdistro/pull/25736
* Contributors: Kevin Hallenbeck

1.0.3 (2020-07-09)
------------------
* Fix cmake dependency error with catkin_EXPORTED_TARGETS
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_aut
* Enabled code coverage testing when built as debug
* Fixed bad asserts
* Contributors: Kevin Hallenbeck

1.0.2 (2018-06-29)
------------------

1.0.1 (2018-06-28)
------------------

1.0.0 (2018-04-25)
------------------
* Use ROS message syncronization to dynamically sync up to 4 units (master and 3 slaves)
* Fixed LCM cmake detection using cmake module FindLCM.cmake
* LCM status messages have 20ms period
* Added roslaunch syntax check
* Added hz test
* Updated license year for 2018
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

0.1.2 (2018-04-11)
------------------
* Fixed inverter status
* Contributors: Eric Myllyoja, Kevin Hallenbeck

0.1.1 (2017-09-07)
------------------

0.1.0 (2017-09-07)
------------------
* Initial release
* Contributors: Eric Myllyoja, Kevin Hallenbeck
