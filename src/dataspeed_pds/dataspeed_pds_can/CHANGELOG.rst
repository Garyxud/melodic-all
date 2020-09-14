^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataspeed_pds_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-07-28)
------------------

1.0.4 (2020-07-24)
------------------

1.0.3 (2020-07-09)
------------------
* Fix cmake dependency error with catkin_EXPORTED_TARGETS
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_aut
* Use fewer function calls to setup message sync (ROS)
* Use fewer function calls to setup message sync (CAN)
* Add argument to enable/disable CAN message filtering on DBW message range
* Enabled code coverage testing when built as debug
* Fixed bad asserts
* Contributors: Eric Myllyoja, Kevin Hallenbeck

1.0.2 (2018-06-29)
------------------

1.0.1 (2018-06-28)
------------------

1.0.0 (2018-04-25)
------------------
* Use ROS message syncronization to dynamically sync up to 4 units (master and 3 slaves)
* Increased can_rx queue size to support all 4 units at the same time
* Added roslaunch syntax check
* Added hz test
* Updated license year for 2018
* Contributors: Kevin Hallenbeck

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
