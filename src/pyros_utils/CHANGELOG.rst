^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyros_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2017-03-21)
------------------
* catkin > 0.2
* Contributors: alexv

0.1.3 (2016-12-22)
------------------
* adding trick to patch import behavior from rospy generated __init_\_ relay.
* now using ros-shadow-fixed in docker for travis
* fixing typo in travis_checks
* now testing with docker. added kinetic test. bumped catkin_pip requirements.
* Contributors: AlexV, alexv

0.1.2 (2016-06-03)
------------------
* fixing travis file to check multiple distros.
* fixing python package version not matching catkin package version.
* Contributors: alexv

0.1.1 (2016-06-02)
------------------
* using renamed dependency catkin_pip
* Contributors: alexv

0.1.0 (2016-06-01)
------------------
* now using shadow-fixed ros packages for travis test build
* cleanup.
* fixing tests
* fixing cmakelists and setup.py. removed requirements.txt
* cleaning up README, removed ros_utils since this is a pure ROS package, like pyros-test
* fixing __init_\_ imports. added travis.yml.
* extracted code from pyros-setup
* Contributors: AlexV, alexv
