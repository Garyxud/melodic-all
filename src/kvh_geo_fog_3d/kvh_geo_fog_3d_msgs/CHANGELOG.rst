^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh_geo_fog_3d_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2020-01-21)
-----------
* No changes to msgs package.

1.3.2 (2020-01-17)
-----------
* No changes to msgs package.

1.3.1 (2020-01-14)
-----------
* No changes to msgs package.

1.3.0 (2019-12-2)
-----------
* Updating packaging and cmakes to conform to catkin_lint
* Merging in all of the PRS preparation. This code should be just about good to go.
* Fixing LICENSE file for msgs.
* Moving license files to be under each package, since theoretically they could be licensed separately.
* Adding license information.
* Contributors: Zach LaCelle

1.2.0 (2019-09-27)
-----------
* Merge branch 'raw_packets' into 'master'
  Filter Options and Magnetometer
  See merge request DART/kvh_geo_fog_3d!14
* Filter Options and Magnetometer
* Removed field that is not supported natively by their library.
* Added message types for raw gnss and raw sensors kvh packets.
* Merge branch 'odom_packet' into 'master'
  Odom packet
  See merge request DART/kvh_geo_fog_3d!13
* Added odom state message and message publishers to kvh driver node.
* Adding message for wheel odometry.
* Contributors: Bostic, Trevor R, LaCelle, Zachary

1.1.0 (2019-08-13 16:32:35 -0400)
---------------------------------
* Adding build/clean scripts for msgs
* Moving messages to their own package
