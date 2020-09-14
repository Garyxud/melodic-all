^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ocean_battery_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.7 (2018-12-17)
------------------
* Merge pull request `#71 <https://github.com/PR2/pr2_power_drivers/issues/71>`_ from Jntzko/last_battery_update
  Initialize last battery update time with 0
* Initialize last battery update time with 0
  in kinetic initializing a ros time with -1 throws the following error:
  "Time is out of dual 32-bit range"
  Change it to time(0) keeps the previous behaviour, but uses time 0 as initial value.
* Contributors: Yannick Jonetzko

1.1.6 (2018-02-11)
------------------
* change maintainer to ROS orphaned package maintaner
* Contributors: Kei Okada

1.1.5 (2015-01-13)
------------------

1.1.3 (2015-01-12)
------------------
