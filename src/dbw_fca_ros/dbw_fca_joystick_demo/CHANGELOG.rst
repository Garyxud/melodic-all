^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_fca_joystick_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2020-07-09)
-------------------
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

1.0.9 (2020-02-14)
------------------
* Add door commands
* Contributors: Kevin Hallenbeck

1.0.8 (2019-10-17)
------------------

1.0.7 (2019-09-13)
------------------

1.0.6 (2019-08-13)
------------------

1.0.5 (2019-07-24)
------------------

1.0.4 (2019-07-11)
------------------

1.0.3 (2019-05-03)
------------------

1.0.2 (2019-03-14)
------------------

1.0.1 (2019-03-01)
------------------

1.0.0 (2018-11-30)
------------------
* Use the ${catkin_EXPORTED_TARGETS} macro for target dependencies
* Removed joystick deadzone
* Added parameters for brake and throttle gains (sanitized from 0 to 1)
* Contributors: Kevin Hallenbeck

0.0.2 (2018-10-23)
------------------
* Added option to command steering torque instead of position
* Added option to enable/disable each command topic
* Contributors: Kevin Hallenbeck

0.0.1 (2018-08-08)
------------------
* Initial release
* Contributors: Kevin Hallenbeck
