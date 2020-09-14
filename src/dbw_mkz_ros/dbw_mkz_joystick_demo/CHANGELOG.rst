^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_mkz_joystick_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.9 (2020-07-09)
------------------
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

1.2.8 (2020-02-20)
------------------

1.2.7 (2020-02-14)
------------------

1.2.6 (2019-11-11)
------------------

1.2.5 (2019-10-30)
------------------

1.2.4 (2019-09-13)
------------------

1.2.3 (2019-08-13)
------------------

1.2.2 (2019-07-24)
------------------

1.2.1 (2019-07-11)
------------------

1.2.0 (2019-05-03)
------------------
* Added angle/torque steering command modes (not supported on all platforms)
* Contributors: Kevin Hallenbeck

1.1.2 (2019-03-14)
------------------

1.1.1 (2019-03-01)
------------------

1.1.0 (2018-11-30)
------------------
* Use the ${catkin_EXPORTED_TARGETS} macro for target dependencies
* Removed joystick deadzone
* Added parameters for brake and throttle gains (sanitized from 0 to 1)
* Contributors: Kevin Hallenbeck

1.0.17 (2018-10-27)
-------------------
* Added option to enable/disable each command topic
* Contributors: Kevin Hallenbeck

1.0.16 (2018-08-29)
-------------------

1.0.15 (2018-08-21)
-------------------

1.0.14 (2018-08-20)
-------------------
* Removed roslaunch check that was failing in indigo due to rviz
* Contributors: Kevin Hallenbeck

1.0.13 (2018-06-06)
-------------------
* Warn and suggest fix for incorrect Logitech gamepad X/D switch configuration
* Removed joystick_demo namespace
* Contributors: Kevin Hallenbeck

1.0.12 (2018-01-30)
-------------------
* Added argument for joystick device path
* Added roslaunch tests
* Contributors: Kevin Hallenbeck

1.0.11 (2017-10-19)
-------------------
* Added missing dependencies
* Handle joystick unplug and re-plugin events
* Contributors: Kevin Hallenbeck

1.0.10 (2017-10-03)
-------------------

1.0.9 (2017-09-19)
------------------
* Changed joystick demo enable buttons default to true
* Contributors: Kevin Hallenbeck

1.0.8 (2017-09-07)
------------------

1.0.7 (2017-08-21)
------------------
* Added steering wheel velocity (svel) parameter
* Contributors: Kevin Hallenbeck

1.0.6 (2017-06-21)
------------------
* Check for expected joystick button and axis count
* Additional dependencies
* Contributors: Kevin Hallenbeck

1.0.5 (2017-04-25)
------------------
* Updated package.xml format to version 2
* Add missing msg dependencies
* Joystick demo cleanup
* Unique target names
* Contributors: Kevin Hallenbeck, P. J. Reed

1.0.4 (2016-12-06)
------------------

1.0.3 (2016-11-17)
------------------

1.0.2 (2016-11-07)
------------------

1.0.1 (2016-10-10)
------------------

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Micho Radovnikovich
