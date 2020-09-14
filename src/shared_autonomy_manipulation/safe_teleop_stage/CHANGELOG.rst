^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package safe_teleop_stage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2019-12-18)
------------------
* add param to plan safe trajectory on backwards (`#13 <https://github.com/SharedAutonomyToolkit/shared_autonomy_manipulation/issues/13>`_)
* add safe_teleop + move_base example
    see https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_fetch_robot/jsk_fetch_startup for detail
    select joystick with safe mode : 'rosservice call /cmd_vel_mux/select /safe_teleop_base/safe_vel'
    select move_base navigation    : 'rosservice call /cmd_vel_mux/select /cmd_vel'
  * fix safe_teleop_stage.launch
* Contributors: Yuki Furuta, Kei Okada

0.0.2 (2017-07-07)
------------------
* change catkin_package() (`#9 <https://github.com/SharedAutonomyToolkit/shared_autonomy_manipulation/issues/9>`_)
* Contributors: Kei Okada

0.0.1 (2017-07-07)
------------------
* add 'ROS Orphaned Package Maintainers' as maintainer (`#6 <https://github.com/SharedAutonomyToolkit/shared_autonomy_manipulation/pull/6>`_)
* safe_teleop_pr2 and safe_teleop_stage do not require safe_teleop_base at
  build time (`#3 <https://github.com/SharedAutonomyToolkit/shared_autonomy_manipulation/pull/3>`_)
* catkinize the packages to work on hydro (`#1 <https://github.com/SharedAutonomyToolkit/shared_autonomy_manipulation/pull/1>`_)
* fixed authors/maintainers in manifest files
* moved to shared_autonomy
* Contributors: Benjamin Pitzer, Ryohei Ueda, Yuki Furuta
