^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2eus_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.14 (2019-02-11)
-------------------
* fix typo and add pr2-init in pr2eus-moveit.l for PR2 + moveit (`#382 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/382>`_ )

  * add moveit pr2-init in pr2eus-moveit.l
    redefine original pr2-init -> pr2-init-org
  * fix parenthesis close in pr2eus-moveit.l

* Fix velocity time scaling (`#377 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/377>`_ )

  * Fix scaling of multi dof joint trajectory
  * scale accelerations in :trajectory-filter
  * fix velocity time scaling
  * Add test for time scaling of vel and acc in :trajectory-filter

* Fix :angle-vector-make-trajectory to ignore start-offset-time (`#365 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/365>`_ )

  * Remove first point of new traj if it overlaps with existing traj.
    Without this fix, traj points having the same time_from_start appear.
    This may harm joint trajectory action server.
  * Use exact time_from_start to pass test
  * Fix :angle-vector-make-trajectory to ignore start-offset-time
  * Reduce start-offset-time to speed up tests
  * Use init-pose instead of reset-manip-pose to speed up test-angle-vector-sequence-motion-plan
  * Loosen time limit of new test
  * Add test for start-offset-time with avs

* Fix time_from_start in motion-planned angle-vector-sequence (`#363 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/363>`_ )

  * Update robot in *ri* in 1st loop
  * Set time_from_start correctly in :angle-vector-make-trajectory
    Previously, traj points for second av started from zero time_from_start
  * Add equal to sort in test to avoid error
  * Add test for concatenation in :angle-vector-make-trajectory

* Fix :trajectory-filter to ignore start-offset-time (`#361 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/361>`_ )

  * Remove start-offset-time from passing args
  * Fix :trajectory-filter to ignore start-offset-time
  * Add test for start-offset-time

* pr2eus_moveit: support motion with mobile base (`#357 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/357>`_ )

  * reset-total-time is used only when trajectory is short
    See https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/0.3.13/pr2eus_moveit/euslisp/robot-moveit.l#L539-L544
  * Don't skip :trajectory-filter as before
    So far, this has been realized by an odd way
    (https://github.com/jsk-ros-pkg/jsk_pr2eus/commit/ca27e9d8dbe765b1879daf9ccd80e09c81674ab1)
  * pr2eus_moveit: run test only when pr2_gazebo is available
  * pr2eus_moveit: support planning with base movement

* run everything within jenkins (`#340 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/340>`_)

  * explictly set DISPLAY="" for roseus test
  * re-define :joint-angle to avoid print violate max/min-angle that exceeds 4M log limit
  * test-pr2eus-moveit.l: not sure why, but sometimes utf-8 code is displayed and brakes catkin build
  * set time-limit for pr2-ri-test to 600
  * .travis.yml: run everything within travis
  * install pr2-arm-kinematics for indigo

* Use service call for collision-object-publisher (`#324 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/324>`_)

  * operate attached_collision_object by service call
  * refactor codes in collision-object-publisher

* Contributors: Affonso Guilherme, Kei Okada, Shingo Kitagawa, Shun Hasegawa, Yuki Furuta

0.3.13 (2017-07-14)
-------------------
* [pr2eus_moveit] add 0.5 seconds sleep after collision object pub (`#315 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/315>`_)
  * add comment why we need unix:sleep
  * add 0.5 seconds sleep after collision object pub

* add test for https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/310#issuecomment-314694668 (`#312 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/312>`_ )
  * test/test-pr2eus-moveit.l, add test-moveit-fastest-trajectory, ensure that :angle-vector-motion-plan will not send faster motion then moveit planned motion
  * display both scaled trajectory time and actual time_to_start time
  * robot-moveit.l : set default start-offset-time to 0, not to skip :trajectory-filter
  * robot-moveit.l, fix debug info (/ total-time 1000) -> (/ total-time 1000.0)

* Revert `#310 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/310>`_ "[pr2eus_moveit] fix typo in total-time condition" (`#314 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/314>`_ )
* Contributors: Kei Okada, Shingo Kitagawa

0.3.12 (2017-07-11)
-------------------
* robot-interface.l: send angle-vector only once, some controller-table had multiple definition for one method, we ignore them (`#308 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/308>`_)
  * test/test-pr2eus-moveit.test only runs on indigo
  * fix when two controller has same action instance
  * add dummy controller for `#308 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/308>`_
  * send angle-vector only once, some controller-table had multiple definition for one method, we ignore them

* [pr2eus_moveit] fix bug in angle-vector-motion-plan (`#309 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/309>`_)
  * fix bug in angle-vector-motion-plan error occur when (length controller-actions) != (length (send self ctype))
    this case happens when you init robot-interface with :default-controller, but send av with :rarm-controller.

* [pr2eus_moveit] fix typo in total-time condition (`#310 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/310>`_)
* [pr2eus_moveit] fix typo in robot-moveit.l (`#306 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/306>`_ )
  * [(:angle-vector-motion-plan] controller-type -> ctype

* Contributors: Kei Okada, Shingo Kitagawa

0.3.11 (2017-06-25)
-------------------
* pr2eus_moveit/euslisp/robot-moveit.l: support tm :fast in :angle-vector-motion-plan (`#297 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/297>`_ )
  * add :scale for :fast in :angle-vector-motion-plan
  * add trajectory_constraints commentout
    trajectory_constraints is not used in motion planning.
    see https://github.com/ros-planning/moveit_msgs/issues/2
  * add max_velocity/acceleration_scaling_factor
  * support tm :fast in :angle-vector-motion-plan

* pr2eus_moveit/euslisp/robot-moveit.l: add angle-vector-sequence-motion-plan test (`#293 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/293>`_ )
  * set longer time-limit for moveit test
* pass ctype in angle-vector-motion-plan (`#292 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/292>`_ )
* advertise CollisionObject with latch=t (`#290 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/290>`_ )
* Contributors: Kei Okada, Shingo Kitagawa

0.3.10 (2017-03-02)
-------------------

0.3.9 (2017-02-22)
------------------
* Support Kinetic (`#284 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/284>`_ )
  * pr2_controllers_msgs is not released on J/K

* [pr2eus_moveit/collision-object-publisher.l] fix bug in :wipe-all (`#283 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/283>`_ )
  * separate wipe-all and fix bug
  * set new hash-table in :clear-all
* [pr2eus_moveit/robot-moveit.l] support angle-vector-sequence with MoveIt! (`#282 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/282>`_ )
  * support angle-vector-sequence for motion plan
  * remove trajectory-constraints for motion plan
* Contributors: Kei Okada, Shingo Kitagawa

0.3.8 (2017-02-07)
------------------

* robot-moveit.l
  * enable set default planner in moveit-env initialization (`#280 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/280>`_ )
  * [pr2eus_moveit] pass start-offset-time as starttime to :send-trajectory (`#276 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/276>`_)
  * pass start-offset-time to :send-trajectory
  * angle-vector-motion-plan return angle-vector (`#268 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/268>`_)
  * angle-vector-motion-plan accepts angle-vector seq
    modify :angle-vector-motion-plan to accept angle-vector-sequence
    set (butlast avs) as TrajectoryConstraints (`#259 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/259>`_)
  * add :ctype args in angle-vector-motion-plan to set controller-type for :angle-vector-motion-plan(`#261 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/261>`_)
  * total-time is msec ,and orig-total-time is sec, :total-time in :trajectory-fiter is msec (`#257 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/257>`_) FIx bugs in `#252 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/252>`_

* collision-object-publisher.l
  * [pr2eus_moveit/collision-object-publisher] support body class object (`#269 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/269>`_)
  * fix bug in collision-object-publisher :wipe-all (`#267 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/267>`_)
  * remove unused key in collision-object-publisher (`#266 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/266>`_)

* Contributors: Naoya Yamaguchi, Kei Okada, Shingo Kitagawa

0.3.7 (2016-11-08)
------------------
* :trajectory-filter add clear-velocities
* Contributors: Kei Okada

0.3.6 (2016-11-02)
------------------
* Update on robot-modeit.l ( `#252 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/252>`_ )
  * :angle-vector-motion send trajectory for joints not incldued in move-arm
  * :trajectory-filter scale based on original time sequences
  * add feature to filter trajectory using total-time
  * robot-moveit.l (:trajectory-filter) add start-offset-time
  * more message on ros-info
  * cleanup :angle-vector-motion-plan function using orig-total-time variable
* Contributors: Kei Okada

0.3.5 (2016-09-16)
------------------

0.3.4 (2016-06-22)
------------------

0.3.3 (2016-05-28)
------------------
* CMakeLists.txt : forget to install euslisp directory ( `#230 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/230>`_ )
* Contributors: Kei Okada

0.3.2 (2016-05-26)
------------------

0.3.1 (2016-05-22)
------------------
* add pr2eus_moveit/README.md
* pr2eus_moveit: add test program
* robot-moveit.l : add info message for mumber of points and duration
* use RRTConnectkConfigDefault as a defualt planner
* robot-moveit.l : fix wrong commit on https://github.com/jsk-ros-pkg/jsk_pr2eus/commit/7d461b7ef199e26f0f9826ed4f1b1fd4cea606fe#commitcomment-17502889
* move pr2eus-moveit -> robot-moveit.l
* pr2eus_moveit: CMakeLists.txt install euslisp/ tutorials/ directory
* pr2eus-moveit.l : fix wrong commit on https://github.com/jsk-ros-pkg/jsk_pr2eus/commit/a55cfb08724ae0034382e2407f60d6830729e04b#commitcomment-17500452
* Contributors: Kei Okada

0.3.0 (2016-03-20)
------------------

0.2.1 (2016-03-04)
------------------

0.2.0 (2015-11-03)
------------------

0.1.11 (2015-06-11)
-------------------

0.1.10 (2015-04-03 18:49)
-------------------------

0.1.9 (2015-04-03 16:52)
------------------------

0.1.8 (2015-02-25)
------------------
* [pr2eus_moveit] package.xml fix version number
* [pr2eus_moveit] Catkinize pr2eus_moveit
* Contributors: Kei Okada, aginika

0.1.7 (2015-02-10)
------------------
* fix typo
* add code for using action-server instead of service
* add check-state-validity service and fix minor bug
* fix bug in collision-object-publisher.l
* change moveit groupname
* add code for using arms
* added eus2scene.l
* add publish-eusscene-marker.l
* add publish-eusscene.l
* Contributors: YoheiKakiuchi, mmurooka, tarukosu

0.1.6 (2014-05-11)
------------------

0.1.5 (2014-05-03)
------------------

0.1.4 (2014-05-02 22:28)
------------------------

0.1.3 (2014-05-02 18:04)
------------------------

0.1.2 (2014-05-01 22:43)
------------------------

0.1.1 (2014-05-01 02:14)
------------------------
* add pr2-tabletop-demo, picking object up on table useing moveit
* comment out debug message
* update pr2eus_moveit for using constraints
* Merge pull request #9 from YoheiKakiuchi/add_use_directly_joint_trajectory
  use joint trajectory mode for moveit
* add clear-world-scene method to pr2eus-moveit
* use joint trajectory mode for moveit
* fix typo :frame_id -> :frame-id
* enable to set object-id with keyword
* update publish-eusobject.l
* add publish-eusobject.l for publishing eus model to moveit environment
* change loading order for pr2eus-moveit
* fix minor bug
* add pr2-moveit.l
* fix typo
* add publish-collision-object
* add make-virtual-joint-constraints
* add :motion-plan-raw method for testing planning
* delete method for attached-object
* add :add-attached-object to collision-object-publisher
* add :query-planner-interface to pr2eus-moveit
* update
* add keyword for adding constraints to motion-plan
* add making constraints functions
* update pr2eus-moveit.l
* add method for robot-interface on pr2eus-moveit
* update pr2eus-moveit
* update pr2eus_moveit tutorials
* add :sync-robot-model method to pr2eus-moveit
* move collision-object-sample.l to tutorials
* add tutorials to pr2eus_moveit
* add :execute-trajectory method to pr2eus-moveit
* update sample for pr2eus_moveit
* add updating faces coords
* add collision-object-sample
* add :relative-pose keyword to collision-object-publisher.l
* fix typo and minor bug
* implement :motion-plan method to pr2eus-moveit.l
* fix typo
* add using torso configuration to pr2eus-moveit.l
* rename pr2eus_moveit.l -> pr2eus-moveit.l
* implement :get-ik-for-pose to moveit-environment
* rename scene-topic -> scene-service
* add pr2eus_moveit.l for using moveit from roseus interface
* add package dependancy to pr2eus_moveit
* move :get-planning-scene method to get-planning-scene function
* add pr2eus_moveit for using moveit components from roseus
* Contributors: Yohei Kakiuchi, YoheiKakiuchi, youhei
