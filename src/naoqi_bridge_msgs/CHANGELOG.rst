^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_bridge_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2018-02-15)
------------------
* delete msg because it can be replaced by std_srvs/SetBool.srv (`#36 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/36>`_)
* delete srv file because it can be replaced by `#37 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/37>`_ (`#39 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/39>`_)
* add get/set string srv files (`#38 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/38>`_)
* Merge pull request `#37 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/37>`_ from kochigami/add-get-set-float-srv
  add get/ set float service
* add get/ set float service
* cleanup package.xml
* Contributors: Kanae Kochigami, Karsten Knese, Natalia Lyubova

0.0.6 (2016-11-08)
------------------
* Merge pull request `#29 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/29>`_ from kochigami/rename-tactile-touch
  [msg] rename TactileTouch to HeadTouch
* [msg] rename TactileTouch to HeadTouch
* Merge pull request `#21 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/21>`_ from kochigami/add-msg-and-srv-for-naoqi-sound-localization
  add srv and msg for naoqi_apps/launch/soundLocalization.launch
* add srv and msg for naoqi_apps/launch/soundLocalization.launch
* Contributors: Kanae Kochigami, Karsten Knese, Natalia Lyubova

0.0.5 (2015-11-15)
------------------
* add orthogonal / tangential security distance service files
* change constants with capital letters
* increase touched hand directions and move it in msg folder correctly
* add back bumper value for Pepper
* Contributors: Kanae Kochigami, Vincent Rabaud

0.0.4 (2015-08-26)
------------------
* Merge pull request `#2 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/2>`_ from vrabaud/master
  add a message concerning the robot information
* add a service to get robot info
* add a message concerning the robot information
* update package.xml with description and author
* Contributors: Karsten Knese, Vincent Rabaud

0.0.3 (2015-07-30)
------------------
* remove legacy msgs
* replace tag in package.xml
* fix double message_runtine dependency
* msg transfer
* fix message_generation vs runtime
* Contributors: Karsten Knese, Vincent Rabaud

0.0.2 (2015-06-10)
------------------
* add confidence
* Merge pull request `#1 <https://github.com/ros-naoqi/naoqi_bridge_msgs/issues/1>`_ from lsouchet/master
  Add Event, PoseWithConfidence and Status msgs.
* Add Event, PoseWithConfidence and Status msgs.
* Contributors: Karsten Knese, karsten1987, lsouchet
