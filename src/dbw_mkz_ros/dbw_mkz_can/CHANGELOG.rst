^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_mkz_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.9 (2020-07-09)
------------------
* Add gear reject enumerations
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

1.2.8 (2020-02-20)
------------------
* Update firmware versions
* Contributors: Kevin Hallenbeck

1.2.7 (2020-02-14)
------------------
* Update firmware versions
* Report NAN for signals that are unavailable/faulted
* Use fewer function calls to setup message sync
* Add Lincoln Aviator to list of platforms
* Contributors: Kevin Hallenbeck

1.2.6 (2019-11-11)
------------------

1.2.5 (2019-10-30)
------------------
* Add steering wheel buttons in Misc1Report
* Contributors: Kevin Hallenbeck, Sun Hwang

1.2.4 (2019-09-13)
------------------
* Added argument to enable/disable CAN message filtering on DBW message range
* Contributors: Kevin Hallenbeck

1.2.3 (2019-08-13)
------------------
* Updated firmware versions
* Updated website maintenance link
* Contributors: Kevin Hallenbeck

1.2.2 (2019-07-24)
------------------
* Extend licensing to each module
* Contributors: Kevin Hallenbeck, Sun Hwang

1.2.1 (2019-07-11)
------------------
* Updated firmware versions
* Added support for non-hybrid brake report values
* Added gear number to throttle info message
* Added throttle and brake limp-home statuses
* Contributors: Kevin Hallenbeck, Sun Hwang

1.2.0 (2019-05-03)
------------------
* Added angle/torque steering command modes (not supported on all platforms)
* Added warnings for unknown and unsupported command types
* Added support for firmware change that uses SVEL resolution of 4 deg/s
* Added FORD_C1 platform
* Added script to estimate the number of wheel counts per killometer
* Added odometer and battery voltage to fuel level report
* Added casts to force single precision floating point math
* Contributors: Kevin Hallenbeck

1.1.2 (2019-03-14)
------------------

1.1.1 (2019-03-01)
------------------
* Updated firmware versions
* Refactored tcpNoDelay() for subscribers
* Added missing tests for PlatformVersion.h
* Contributors: Kevin Hallenbeck

1.1.0 (2018-11-30)
------------------
* Updated firmware versions
* Removed all BOO control options and manually implemented auto BOO control for legacy firmware (brake lights)
* Added BTYPE (brake type) bit
* Added CMD_DECEL brake command type (only for non-hybrid platforms)
* Replaced dbw_mkz_twist_controller with dataspeed_ulc_can in dbw.launch
* Added throttlePercentFromPedal lookup table function and corresponding test
* Use the ${catkin_EXPORTED_TARGETS} macro for target dependencies
* Added DriverAssistReport message
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.17 (2018-10-27)
-------------------
* Updated firmware versions
* Updated list of platforms
* Disengage on any fault for brake/throttle/steering (change AND to OR)
* Added outside air temperature to Misc1Report
* Latch firmware version on any change (previously only latched once)
* Changed pedal_luts default from true to false (forward command type by default now)
* Fixed handling of all the firmware/module requrements for brake command type CMD_TORQUE_RQ
* Disregard overrides on unused subsystems using the TIMEOUT bit
* Fixed typo in nodelets.xml of dbw_mkz_can
* Finished unit tests of PlatformMap
* Use sign of wheel speeds to set sign of vehicle speed, fixes issue #24
* Set CXX_STANDARD to C++11 only when necessary
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.16 (2018-08-29)
-------------------
* Force compiler to use C++11
* Handle version message with a map/database of several platform/module combinations
* Implemented firmware version requirements for forwarding pedal command type
* Contributors: Kevin Hallenbeck

1.0.15 (2018-08-21)
-------------------
* Updated firmware versions
* Contributors: Kevin Hallenbeck

1.0.14 (2018-08-20)
-------------------
* Updated firmware versions
* Enabled code coverage testing when built as debug
* Increased the steering command range to +-INT16_MAX, the specific range limit is applied by the firmware
* Skip warning about brake and throttle commands when the reserved bit is set
* Match CAN messages 0x060 to 0x07F even though some are unused
* Capitalized the COUNT field to match code style
* Added parameter for local/embedded pedal LUTs
* Added option to forward higher level pedal commands (percent/torque) to the embedded modules
* Added unit tests for exported header files
* Extracted sonar color to a separate function
* Added missing include
* Contributors: Kevin Hallenbeck

1.0.13 (2018-06-06)
-------------------
* Updated firmware versions
* Added option to enable/disable warnings on received command messages
* Added support for the RES+ and RES- buttons
* Added explicit casts to float
* Added firmware version of separate shifting module
* Contributors: Kevin Hallenbeck

1.0.12 (2018-01-30)
-------------------
* Updated firmware versions
* Moved ModuleVersion class and look-up-tables to exported header files (for use by other packages)
* Added power fault bit to report when modules lose power
* Added missing warning about steering fault preventing enable
* Added roslaunch argument to set use_sim_time or not
* Added roslaunch tests
* Only warn once for each unknown module version
* Contributors: Kevin Hallenbeck

1.0.11 (2017-10-19)
-------------------
* Updated firmware versions
* Added missing dependencies
* Contributors: Kevin Hallenbeck

1.0.10 (2017-10-03)
-------------------
* Updated steering firmware version
* Renamed feature name
* Contributors: Kevin Hallenbeck

1.0.9 (2017-09-19)
------------------
* Added warning to update old firmware
* Added link to request a license
* Added more detail to fault warnings
* Contributors: Kevin Hallenbeck

1.0.8 (2017-09-07)
------------------
* Migrated from dataspeed_can_msgs to can_msgs
* Contributors: Kevin Hallenbeck

1.0.7 (2017-08-21)
------------------
* Removed steering report driver activity bit
* Replaced connector fault with timeout, and warn on timeout
* Keep track of module firmware versions
* Added gear rejection enumeration to gear report
* Added licensing and VIN
* Added wheel positions report (replaces suspension report)
* Added option to use buttons for enable/disable, or not
* Added enable button combination for Mondeo without ACC (set_dec and cc_res)
* Added steering wheel left D-Pad buttons
* Updated ackermann steering parameters (including steering ratio)
* Prioritize the local include folder (there were issues with catkin workspace overlays)
* Fixed accel orientation to match the ROS standard
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.6 (2017-06-21)
------------------
* Added frame_id parameter for IMU and Twist messages
* Properly handle IMU unknown covariance and fields that are not present
* Removed SuspensionReport (data was unintelligible)
* Reorganized launch files.
* Swapped lateral and longitudinal acceleration in IMU message.
* Export dispatch.h for use by other packages
* Added clear bit to command messages
* Updated nodelet to the PLUGINLIB_EXPORT_CLASS macro
* Additional dependencies
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.5 (2017-04-25)
------------------
* Updated package.xml format to version 2
* Unique target names
* Contributors: Kevin Hallenbeck

1.0.4 (2016-12-06)
------------------
* Added brake and throttle thrashing scripts to try and induce faults
* Changed wheel speeds to signed values
* Contributors: Kevin Hallenbeck, Joshua Whitley

1.0.3 (2016-11-17)
------------------
* Added QUIET bit to disable driver override audible warning
* Print brake/throttle/steering firmware versions
* Handle and report steering faults (FLTBUS1 and FLTBUS2)
* Contributors: Kevin Hallenbeck

1.0.2 (2016-11-07)
------------------
* Configurable steering ratio
* Contributors: Kevin Hallenbeck

1.0.1 (2016-10-10)
------------------
* Added support for apt-get binary packages
* Added twist message computed from vehicle speed and steering wheel angle.
* Contributors: Kevin Hallenbeck

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Micho Radovnikovich
