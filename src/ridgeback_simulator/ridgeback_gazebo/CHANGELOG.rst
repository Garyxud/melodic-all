^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ridgeback_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2020-08-24)
------------------
* Apply the config argument properly
* Move spawning the ridgeback into a separate file for use with the new sim environments. Enable teleoperation by default
* Add Robot Spawn Pose Launch Args
  Adds x,y,z,yaw arguments to set pose where the robot spawns.
  New arguments are defaulted to their previous values.
  Example:
  ```
  roslaunch ridgeback_gazebo ridgeback_world.launch x:=10 y:=10
  world_name:=/path/to/my/world.sdf
  ```
* Contributors: Alex Moriarty, Chris Iverach-Brereton

0.1.0 (2019-07-22)
------------------

0.0.3 (2018-04-26)
------------------

0.0.2 (2017-05-08)
------------------
* Changed Ridgeback config to environment variable and minor clean-up of ForceBasedPlugin.
* Updated default world to a wider and less constrained version.
* Contributors: Tony Baltovski

0.0.1 (2016-05-25)
------------------
* Initial release.
* Contributors: Mike Purvis, Tony Baltovski
