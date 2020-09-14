^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package power_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2019-08-28)
------------------
* Merge pull request `#10 <https://github.com/fetchrobotics/power_msgs/issues/10>`_ from pavansoundara/battery-data

  We've released a binary version of the drivers which publishes the this version
  Note: will investigate switching both internal and external code to use:
  http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html in the future.

  Adds:

  * total_capacity
  * current_capacity
  * battery_voltage
  * supply_voltage
  * charger_voltage

  Removes:

  * errors

* Contributors: Alex Moriarty, Pavan Soundara

0.3.0 (2018-07-10)
------------------
* updates ownership
* Merge pull request `#8 <https://github.com/fetchrobotics/power_msgs/issues/8>`_ from cjds/errors
  Added error messages to the battery state for more information
* added error messages to the battery state for more information in FC
* Merge pull request `#6 <https://github.com/fetchrobotics/power_msgs/issues/6>`_ from macmason/master
  Add LICENSE file.
* Add LICENSE file.
* Contributors: Carl Saldanha, Mac Mason, Michael Ferguson, Russell Toris, cjds

0.2.0 (2015-07-06)
------------------
* add message for battery state
* Contributors: Michael Ferguson

0.1.3 (2015-02-24)
------------------
* add name to breaker state message (for multiple breakers in an array)
* Contributors: Michael Ferguson

0.1.2 (2015-01-26)
------------------
* add limits, documentation (breaks MD5)
* Contributors: Michael Ferguson

0.1.1 (2015-01-25)
------------------
* add BreakerCommand
* Contributors: Michael Ferguson
