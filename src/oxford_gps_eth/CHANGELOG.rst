^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oxford_gps_eth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2020-08-07)
------------------
* Changes to make ntrip_forwarding.py support both Python 2 and Python 3
* Contributors: Micho Radovnikovich

1.2.0 (2020-07-06)
------------------
* Use setuptools instead of distutils for python
  http://wiki.ros.org/noetic/Migration#Setuptools_instead_of_Distutils
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Add flexibility to change GGA topic and broadcast IP for NTRIP forwarding
* Fix local frame covariance calculation
* Use cached standard deviation values instead of raw packet values because those values are multiplexed into several packets
* Add quality check of standard deviation values
* Print debugging info with ROS_DEBUG()
* Change BUILD_ASSERT() to std_assert()
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.1.1 (2019-10-31)
------------------
* Install Python script
* Add missing rospy dependency
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.1.0 (2019-10-28)
------------------
* Add capability to forward RTCM messages from a NTRIP caster to OxTS receivers
* Contributors: Micho Radovnikovich

1.0.0 (2018-09-07)
------------------
* Use C++11 or newer
* Publish a sensor_msgs/TimeReference message with GPS time converted to UTC
* Publish a position type string more detailed than the NavSatStatus in the fix message
* Added UTM position and heading to odometry output; Odometry twist now in local frame
* Contributors: Micho Radovnikovich, Kevin Hallenbeck

0.0.6 (2018-03-26)
------------------
* Changed default listen address from broadcast to any
* Added unit tests and rostests
* Added launch file
* Contributors: Kevin Hallenbeck

0.0.5 (2017-08-10)
------------------
* Fixed velocity utm east-north-up orientation
* Properly handle unknown covariance and fields that are not present
* Updated license for year 2017
* Updated package.xml format to version 2
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

0.0.4 (2016-04-11)
------------------
* Updated license year for 2016
* Contributors: Kevin Hallenbeck

0.0.3 (2015-12-21)
------------------
* Added fix for Ubuntu Saucy
* Contributors: Kevin Hallenbeck

0.0.2 (2015-12-14)
------------------
* Added fix for Ubuntu Saucy
* Contributors: Kevin Hallenbeck

0.0.1 (2015-12-10)
------------------
* Ready for public release
* Contributors: Kevin Hallenbeck
