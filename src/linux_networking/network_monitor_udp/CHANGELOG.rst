^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package network_monitor_udp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.16 (2019-11-08)
-------------------
* ROS-melodic .debs for network_monitor_udp incomplete (`#3 <https://github.com/pr2/linux_networking/issues/3>`_)
  * update melodic-devel to include appropriate files
  - python modules were missing (needed setup.py and `catkin_python_setup`) from .deb
  - python executables were missing (needed `catkin_install_python`) from .deb
  - python package `roslib` is deprecated; `rostime` now lives in `rospy`
  - compiled binaries and .launch file were missing from from .deb
* Contributors: David Feil-Seifer, bk-mtg

1.0.15 (2019-03-19)
-------------------

1.0.13 (2019-03-18)
-------------------
* fixed unsigned char errors for melodic compile
* Contributors: David Feil-Seifer
* updated cmake for message build dependency issue in kinetic
* Contributors: David Feil-Seifer

1.0.12 (2019-02-26)
-------------------
* changes to cmakelists files to make packages compile in kinetic
* Contributors: David Feil-Seifer

1.0.9 (2014-10-14)
------------------

1.0.8 (2014-10-10)
------------------
* Removed rosbuild files
* Contributors: TheDash

1.0.7 (2014-10-10)
------------------

1.0.6 (2014-10-10)
------------------

1.0.5 (2014-10-06)
------------------

1.0.4 (2014-10-03)
------------------

1.0.3 (2014-10-03)
------------------

1.0.2 (2014-10-03)
------------------
