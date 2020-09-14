^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_fca_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2020-07-09)
-------------------
* Add gear reject enumerations
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Add sensor reports for wiper, highbeam, and hazard light
* Add sensor reports for gyro, accelerometer, GPS, and tire pressure
* Contributors: Kevin Hallenbeck, Sreedevi Adukkathayar, Sun Hwang

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
* Added support for non-hybrid brake report values
* Added gear number to ThrottleInfoReport
* Contributors: Kevin Hallenbeck, Sun Hwang

1.0.3 (2019-05-03)
------------------
* Updated maximum steering wheel velocity in command message
* Added fuel level report message
* Contributors: Kevin Hallenbeck

1.0.2 (2019-03-14)
------------------

1.0.1 (2019-03-01)
------------------

1.0.0 (2018-11-30)
------------------
* Added CMD_DECEL brake command type (only for non-hybrid platforms)
* Changed maximum brake torque command from 3412 Nm to 5000 Nm
* Contributors: Kevin Hallenbeck

0.0.2 (2018-10-23)
------------------
* Added cruise control buttons
* Removed cruise control related buttons that are not implemented by firmware at this time
* Removed steering debug message
* Contributors: Kevin Hallenbeck

0.0.1 (2018-08-08)
------------------
* Initial release
* Contributors: Kevin Hallenbeck
