^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2eus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.14 (2019-02-11)
-------------------
* [pr2eus] add :get-grasp-result methody and :wait key to :start-grasp method (`#386 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/386>`_ )
* pr2-interface: Make use of return value of :move-gripper method (`#364 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/364>`_ )

  * pr2-interface: make use of return value for :stop-grasp
  * pr2-interface: fix bug on :start-grasp return value of :joint-angle
  * pr2-interface: make use of return values of :move-gripper

* [pr2eus/speak.l] enable `speak-jp` with `wait: t` (`#369 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/369>`_ )
* add test to reproduce `#366 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/366>`_ (play-sound could not play file format) (`#371 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/371>`_ )

  * fix speak-test-action.py, more test on SAY, PLAY_FILE and buiding files
  * [pr2eus/speak.l] use (namestring) to get string from pathname, Fixed `#366 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/366>`_
  * add test-speak-number / test-ri-speak-number
  * add test to reproduce `#366 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/366>`_ (play-sound could not play file format)

* pr2-interface.l: Add optional key arguments for :angle-vector-with-constraint (`#380 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/380>`_ )

  * add :revert-if-fail, :initial-angle-vector, :div optional key for :angle-vector-with-constraint
  * fix minor indent

* robot-interface.l: remove old timer-job (`#321 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/321>`_ )
* fix speak-test.test to fail on volume==0 (`#379 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/379>`_ )

  * [pr2eus/speak.l] Fixed volume insertion checking with :volume accessor exists
  * [pr2eus/speak.l] add :volume keyward for play-sound for sound_play >= 0.3.1 (Closes `#368 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/368>`_)
  * fix speak-test.test to fail on volume==0
    pr2eus/test/speak-test.py: check if SoundRequest has volume attribute

* .travis.yml : remove hydro/jade, add melodic, fix speak-test for indigo (`#385 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/385>`_ )

  * test:speak-test.l add (ros::sleep 2) at the end of script, to wait for last message actually send out
  * fix test for indigo

* Goal header stamp should be the time message is made  (`#358 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/358>`_ )

  * fix goal header stamp time

* [pr2eus/robot-interface.l] Modified warning message in case of we can not connect to follow joint trajectory server (`#381 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/381>`_ )
* fix typo in error message on pr2eus/pr2-interface.l ( `#384 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/384>`_ )
* pr2eus: override clear-costmap namespace for pr2 (`#343 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/343>`_ )

  * pr2eus: override clear-costmap namespace for pr2
  * pr2eus: delegate costmap functions to robot-move-base-interface
  * pr2eus: default value of inflation range: 0.3

* add test for pr2/speak.l (`#374 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/374>`_ )

  * add test for action interface
  * add speak-test.py to check contents of SpeakRequest message
  * run speak-test.l and catch message by hztest

* Equalize min-time behavior of end-coords-interpolation to usual angle-vector (`#355 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/355>`_ )

  * Equalize min-time behavior of end-coords-interpolation to usual angle-vector
  * Add test to catch (`#354 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/354>`_)
  * Add end-coords-interpolation test


* pr2eus_moveit: support motion with mobile base (`#357 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/357>`_ )
* pr2-ri-test.l : add test to check :wait-interpolation, with timeout (`#352 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/352>`_ )

  * fix: pr2-ri-test.l: check return value of :wait-ingterpolation as list
  * fix: Calling (load-ros-manifest pr2eus) for the package without msg/srv will be deprecated
  * pr2eus: robot-interface: add :base-controller-joint-names option
  * clean up :interpolatingp of robot-interface, use :interpolatingp in controller-actions class
  * fix :interpolationp of controller-action-client when using real robot
  * pr2-ri-test.l : add test to check return of :wait-interpolation with timeout value
  * fix :wait-interpolation of pr2-interface, do not wait for moving joints when timeout is not equal to 0
  * reduce the retry to 1, 5(retry) x 600 sec exceeds 50min limit of Travis
  * pr2-ri-test.l : add test to check :wait-interpolation, with timeout

* euscollada 0.4.0 requires changes in tests (from = to eps=, but seems the output is both 0.0) (`#360 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/360>`_ )

  * not sure why we do not need this until now, but we need to use eps=

* Fix typo. stll -> still (`#342 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/342>`_)
* Fix wrong behaviors in :go-pos-unsafe / :move-trajectory / :move-trajectory (`#336 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/336>`_)

  * fix: go-pos-unsafe less than expected if msec < 1000
  * fix: move-trajectory wrong document / implementation

* Fix many typos (`#337 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/337>`_)
* run everything within jenkins (`#340 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/340>`_)

  * explictly set DISPLAY="" for roseus test
  * re-define :joint-angle to avoid print violate max/min-angle that exceeds 4M log limit
  * test-pr2eus-moveit.l: not sure why, but sometimes utf-8 code is displayed and brakes catkin build
  * set time-limit for pr2-ri-test to 600
  * .travis.yml: run everything within travis
  * install pr2-arm-kinematics for indigo

* increase stall_velocity_threshold to pass grasp test (`#338 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/338>`_)

  * increase stall_velocity_threshold to pass grasp test
  * add pr2_gazebo and robot_state_publisher to test_depend
  * skip pr2eus_moveit from test
  * set DISPLAY='' when gui is false
  * hydro/indigo run on travis, jade/kinetic run on jenkins

* [pr2eus] do not pass :wait-until-update in :state args (`#333 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/333>`_ )

  * add comment for :state :wait-until-update
  * add :wait-until-update test
  * pass args not including :wait-until-update keys

* Fix :end-coords-interpolation problems (`#325 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/325>`_ )

  * Adapt end-coords-interpolation for over-360 deg turns
  * Add :steps to :end-coords-interpolation
  * Changes to :end-coords-interpolation

* pr2eus: partially rever speak function (`#332 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/332>`_ )

  * sound_play could not run within travis/jenkins, so use hztest a dummy subscriber
  * pr2eus: integrate speak function, add :play-sound :speak-en, :speak-jp method to robot-interface
  * Revert "pr2eus: add text-to-spech method to robot-interface (`#318 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/318>`_)"
    This reverts commit ecb2a1e29d2d56ae16035064c91617f2b0afa786.

* pr2eus: add text-to-spech method to robot-interface (`#318 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/318>`_)

  * pr2eus: cleanup speak.l
  * pr2eus: robot-interface.l: add text-to-speech methods to robot-interface
  * pr2eus: update test for speak
  * pr2eus: migrate text-to-speech to robot-interface

* fix sub-angle-vector when diff is over 640 (`#323 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/323>`_ )

  * mod 360 to suport rotation over 640
  * add test to chcek sub-angle-vector over 620
  * fix :publish-joint-state after updating angle-vecgtor in robot-interface-simulation-callback, also changed to set av as the keypose

* Contributors: Affonso Guilherme, Kei Okada, Shingo Kitagawa, Shun Hasegawa, Yuki Furuta, Hitoshi Kamada, Iori Yanokura

0.3.13 (2017-07-14)
-------------------
* [pr2eus] enable controller-type in :cancel-angle-vector (`#313 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/313>`_)
  * fix typo in robot-interface ( doc string of :cancel-angle-vector method)
  * cancel angle-vector by controller-type
* Contributors: Kei Okada, Shingo Kitagawa

0.3.12 (2017-07-11)
-------------------
* [robot-interface.l] :angle-vector-duration add document to how we use :max-joint-velocity (`#305 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/305>`_ )
* Contributors: Kei Okada

0.3.11 (2017-06-25)
-------------------
* use make-caemra-from-ros-camera-info-aux inroseus, in order to generate pr2 model corresponding to `jsk-ros-pkg/jsk_roseus/pulls/#526 <https://github.com/jsk-ros-pkg/jsk-ros-pkg/jsk_roseus/pulls/526>`_ (`#301 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/301>`_)
  * [pr2eus/pr2.l] update make-camera-from-ros-camera-info-aux
  * add comment to why we redefine make-camera-from-ros-camera-info-aux in robot model
  * skip position test in test-cameras on hydro

* [robot_interface.l] add tms comment to :angle-vector-sequence c.f. https://github.com/jsk-ros-pkg/jsk_robot/pull/791#pullrequestreview-45324124 (`#299 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/299>`_)
* [robot_interface.l] add :stamp method for reading latest stamp (`#298 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/298>`_)
* .travis.yml: re-enable pr2-ri-test (using gazebo) for indigo (`#296 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/296>`_
  * pr2-ri-test.l: add test to check :wait-for-interpolation, see (`https://github.com/start-jsk/jsk_apc/issues/2106 <https://github.com/start-jsk/jsk_apc/issues/2106>`_)
  * when unknown goal is received, we assume the original goal is canceled and set time-to-finish to 0.0
  * test-start-grasp: send move-gripper with more gain
  * .travis.yml: re-enable pr2-ri-test (using gazebo) for indigo

* [pr2eus][pr2-interface.l] add switch-controller methods  (`#295 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/295>`_
  * [pr2eus] add pr2_mechanism_msgs to depend

* [pr2eus][pr2eus_moveit] use ctype in :send-trajectory and pass ctype in angle-vector-motion-plan (`#295 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/295>`_)
  * use only controller-type in send-trajectory

* [pr2eus] fix some funcs that break behaviors written at docs (`#289 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/289>`_)
  * [pr2eus][default-ri-test.l] fix: load path for passing test on local machine
  * [pr2eus][pr2-ri-test-simple.l] assert return values of robot-interface methods
  * [pr2eus][robot-interface.l] implement :go-waitp when simulation-modep is t
  * [pr2eus][robot-interface.l] :move-to-wait returns t when simulation-modep
  * [pr2eus][robot-interface.l] implement :interpolatingp when :simulation-modep is t

* use link-list instead of (car link-list) in use-base condition(`#272 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/272>`_)
* Contributors: Yuki Furuta, Kei Okada, Shingo Kitagawa, Yohei Kakiuchi, Chi Wun Au

0.3.10 (2017-03-02)
-------------------
* [pr2eus][pr2-interface.l] move move-to / go-pos callback for simulation to robot-interface.l (`#288 <https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/288>`_)
+ [pr2eus] fix: remove the first '/' from frame (`#287 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/287>`_)
* fix: use movebaseaction name for clear-costmap (`#286 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/286>`_)
  * [pr2eus/robot-interface.l] fix: use move-base-action name for clear-costmap
  * [pr2eus][robot-interface.l] soft tab
* Contributors: Kei Okada, Yuki Furuta

0.3.9 (2017-02-22)
------------------
* cleanup CMakeLists.txt, use PR2_CONTROLLERS_MSGS_PACKAGE variable and add geneus for hydro (`#285 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/285>`_ )
* Support Kinetic (`#284 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/284>`_ )
  * need to add geneus for hydro? https://s3.amazonaws.com/archive.travis-ci.org/jobs/203074134/log.txt
  * robot-init-test.l: disable test for jade/kinetic, which did not load pr2-interface.l, beacuse of missing pr2_controller_msgs
  * CMakeLists.txt: using PR2_CONTROLLERS_MSGS_PACKAGE variable to control find_package does not work on hydro
  * pr2-interface.l exits without error on kinetic
  * pr2_controllers_msgs is not released on J/K
  * pr2eus/CMakeLists.txt: pr2_controllers_msgs is not released on J/K
* Contributors: Kei Okada

0.3.8 (2017-02-07)
------------------
* add end-coords-interpolation (`#237 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/237>`_ )
  * Fix typos in :angle-vector (if end-coords-interpolation
  * Force end-coords-interpolation to go to given av
  * add end-coords-interpolation in :angle-vector with:end-coords-interpolation t:  move robot in cartesian space interpolation
* add more message on kinematics simulator mode
* Contributors: Kei Okada, Shun Hasegawa

0.3.7 (2016-11-08)
------------------
* [pr2eus/pr2-interface.l] add :force-assoc option for :start-grasp
* robot-interface.l: send-trajectory-each : check if vels/effs is #f()
* Contributors: Kei Okada, Yuki Furuta

0.3.6 (2016-11-02)
------------------
* add :base-controller-action-name for robot does not have move_base_trajectory_action (`#253 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/253>`_ )
* [pr2eus/robot-interface.l] update actionlib name of default controller. (`#250 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/250>`_ )
* Contributors: Kei Okada, Masaki Murooka

0.3.5 (2016-09-16)
------------------

* robot-interface.l

  * fix :wait-intepolation-smooth for SinglePointJointAcionGoal (`#245 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/245>`_)
  * use control_msgs/FollowJointTrajectoryAction for base trajectory action (`#237 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/237>`_)
  * fix: wrong code in  :move-trajectory (`#240 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/240>`_)
  * the implementation of condition to break loop in :wait-until-update-all-joints. (`#239 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/239>`_)
  * :wait-until-update-all-joints need to call :robot-interface-simulation-callback explicitly (`#238 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/238>`_)

* sometines :state .. :wait-unitl-update t did not return (https://github.com/jsk-ros-pkg/jsk_robot/pull/627)

  * add test-state-wait-until-updatee (`#238 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/238>`_)
  * include also redundant links when calculate collision

* speak.l

  * add *speak-timeout* param to wait action server (`#246 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/246>`_)
  * use single speak-action-client (`#241 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/241>`_)

* CMakeLists.txt: remove unused variable from catkin_package (`#243 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/243>`_)
* pr2.l: comment out pr2 function for pr2-robot (`#242 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/242>`_)

* Contributors: Kei Okada, MasakiMurooka, Yuki Furuta, Chi Wun Au

0.3.4 (2016-06-22)
------------------
* Merge pull request `#235 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/235>`_ from k-okada/fix_smooth
  fix :wait-interpolation-smooth for pr2_controllers_msgs/JointTrajectoryActionFeedback
* add code when last-feedback-msgs-stamp is not updated
* robot-interface.l : wait for feedback message is updated
* fix :wait-interpolation-smooth for pr2_controllers_msgs/JointTrajectoryActionFeedback
* Contributors: Kei Okada

0.3.3 (2016-05-28)
------------------

0.3.2 (2016-05-26)
------------------
* fix typo topuc -> topic
* robot-interface.l : add option to set queue size for /joint_state subscriber
* robot-interface.l : need a consistency of controller order in the the entry of controller-table fix `#227 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/227>`_
* Contributors: Kei Okada

0.3.1 (2016-05-22)
------------------
* [pr2eus/pr2-utils.l] add start-grasp, stop-grasp for *pr2*
* [pr2eus/test/robot-init-test.*, pr2eus/CMakeLists.txt] Add robot-init function rostest. Add rostest execution for it in CMakeLists.txt.
* [package.xml] Add setting for robot-init to package.xml using export tag and rospack plugin functionality (http://wiki.ros.org/pluginlib).
* [pr2eus/robot-interface.l] Add robot-init function. Add documentation string for it.
* [pr2eus/robot-interface.l] wait /clock publish for a while when /use_sim_time is true
* Contributors: Kamada Hitoshi, Shunichi Nozawa, Yuki Furuta

0.3.0 (2016-03-20)
------------------

* add robot-move-base-interface class

  * [robot-interface.l] fix clear-costmap/change-inflation-range to support different move_base node name
  * [robot-interface.l,pr2-interface.l] move clear-costmap and hcange-inflation-range from pr2-interface.l to robot-interface.l
  * [robot-interface.l] check if move-base-trajectory-action is available
  * [robot-interface.l,pr2-interface.l] move odom-callback to robot-move-base-interface class
  * [robot-interface.l] enable to set base_footprint name
  * [test/pr2-ri-test-simple.l] add test for move-to

* Contributors: Kei Okada

0.2.1 (2016-03-04)
------------------

* add robot-move-base-interface, which support move_base interface (`#208 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/208>`_)

* [pr2eus/pr2-interface.l] default argument of change-inflation-range 0.55 -> 0.2 according with the change of default value https://github.com/jsk-ros-pkg/jsk_robot/pull/535 (`#204 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/204>`_)

* add :state :gripper method (`#190 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/190>`_)

  * [pr2eus/pr2-interface.l] add :state :gripper method to fetch information of gripper
  * [pr2eus/robot-interface.l] add :gripper virtual method; :state :gripper accessor to :gripper

* fix `#179 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/179>`_

  * [pr2eus/robot-interface.l] add variable to change default look-all behavior on draw-objects
  * [pr2eus/robot-interface.l] add option :look-all when :draw-objects

* [pr2eus/pr2-interface.l] fix gripper method (`#201 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/201>`_)
* [pr2eus/pr2-interface.l] add document of :gripper method (`#199 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/199>`_)

* [pr2eus/robot-interface.l, pr2eus/pr2-interface.l] fix: :wait-interpolation returns :interpolatingp on real robot (`#191 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/191>`_)

  * [pr2eus/pr2-interface.l] :wait-interpolation returns results of :interpolatingp of controllers on real robot
  * [pr2eus/robot-interface.l] :wait-interpolation returns results of :interpolatingp of controllers on real robot

* [pr2eus] add :go-waitp (`#196 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/196>`_)

* add :effort-vector for reading effort of joint_states (`#188 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/188>`_ )

  * [pr2eus/robot-interface.l] Revert https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/188 and fix :torque-vector to return joint torques.

* update speak command

  * [speak.l] add default variable for waiting speak
  * [speak.l] add speak backward compatibility
  * [test/speak-test.test] add test for speak.l

* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Yuki Furuta, Hitoshi Kamada

0.2.0 (2015-11-03)
------------------
* Bug Fixes

  * [robot-interface.l] change-inflation-range to use new service name (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/169)
  * :interpolating-smoothp not working (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/158)

    * [pr2eus/robot-interface] fix to work :wait-interpolation-smooth  (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/159)
    * test/default-ri-test.l: add test code for :wait-interpolation-smooth,
    * mv default-ri-test.launch-> default-ri-test.test, and add to CMakeLists.txt


* Add :go-* prototype functions  (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/164, https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/171)

  * robot-interface.l: use error instead of warn for :go-* prototype  functions (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/163)
  * [pr2eus/pr2-interface.l] fix return value of `:go-pos-unsafe-wait` along with (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/164)
  * [pr2eus/robot-inferface.l] clarify return value policy (https://github.com/k-okada/jsk_pr2eus/pull/5)
  * [pr2eus] fix go-pos-unsafe
  * pr2-interface.l: add :go-pos-unsafe, :go-pos-unsafe-no-wait, :go-pos-unsafe-wait
  * robot-interface.l: add go-* function prototype
  * pr2-interface.l : addk go-pos-no-wait and go-wait

* Support go-pos-no-wait in simulation mode

  * Display objects in simulationp (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/168)

    *   [robot-interface.l]: (send self :objects objs) should call even in simulationp
    *   [test/default-ri-test.l] add test for :objects methods

  * Fix :move-to in sim mode (check frame-I'd) add test for :move-to (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/167)

    * [pr2-interface.l] move to relative to current position only if frame-id argument is /base_footprint
    * [test/pr2-ri-test-simple.l] add test for move-to

  * Support move-to-no-wait in simplationp (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/165)

   * [pr2-interface.l] :move-to-send , for simulation mode, do not try to call :lookup-transform
   * [pr2-interface.l] fix typo : if -> when, return-from :move-to -> return-from :move-to-send, https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/165#discussion_r37421484
   * [test/pr2-ri-test-simple.l] add test for go-pos, go-pos-no-wait, go-wait
   * [pr2eus/pr2eus/pr2-interface.l] fix typo (short modify) @h-kamada
   * test/test-ri-test.l: :wait-interpolation retuns a list of :interpolationg
   * pr2-interface : support timer-based motion for :move-to
   * more realistic simulation mode

* use default pr2_description (https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/149)

  * [pr2eus] change pr2 camera frame namespace from /openni to  /kinect_head (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/153)

* Other New Features

  * [pr2eus/robot-interface.l] add method :find-object to  robot-interface and test code (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/180)

* Misc Updates

  * [pr2eus/CMakeLists.txt]: remove old groovy codes
  * [pr2eus/speak.l] refactor speak.l (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/176)
    - super easy to read code
    - support wait and timeout for every speaking
    - support multi language with google engine
  * pass additional-weight-list when calling super class method (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/148)
  * [pr2ues/robot-interface.l] check length of avs and tms in  :angle-vector-sequence; add test code (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/151)

* Contributors: Kamada Hitoshi, Kei Okada, Masaki Murooka, Yuki Furuta, Yuto Inagaki

0.1.11 (2015-06-11)
-------------------
* [pr2eus] Print warning message if controller-timeout is nil in robot-interface
* [robot-interface.l] do not raise error when controller have wrong joint name
* [test/pr2-ri-test-simple.l] add test for wrong controller
* Revert "[pr2eus] Use get-topics in speak.l to check whether already advertised or not"
  This reverts commit 134353868b4e826a8a879bb3ac3b9dcbb500a7da.
* [robot-interface.l] update joint in (*ri* . robot) only in controller-type
* [robot-interface.l] update only cotroller joint for simulation mode
* [robot-interface.l] add documents for public methods
* [robot-interface.l] :angle-vector-sequence use default if nil ctype was passed
* [robot-interface.l] :angle-vector use default if nil ctype was passed
* [pr2eus] Use get-topics in speak.l to check whether already advertised or not
* [pr2eus/CMakeLists.txt] add eusdoc
* [pr2eus] remove old manifest.xml
* [pr2eus] Fix :interpolatingp by using ros::*simple-goal-state-active* instead of actoinlib_msgs::GoalStatus::*active*
* [pr2eus] Support ctype in :interpolatingp
* add publish-joint-state and update viewer for the last pose in angle-vector-sequence
* [robot-interface.l] add zero div check
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Yuto Inagaki, Shintaro Noda

0.1.10 (2015-04-03)
-------------------
* [robot-interface.l, pr2-interface.l] support :fast in :angle-vector-sequence
* Contributors: Yuto Inagaki

0.1.9 (2015-04-03)
------------------
* [robot-interface.l] :min-time=0.0 in :angle-vector-sequence because smooth  angle-vector may have short duration for each angle-vector
* [jsk_pr2eus] FIx :angle-vector-sequence by passing ctype argument to :angle-vector-duration
* [pr2-interface.l] remove unused service call '/move_base_node/clear_unknown_space'
* [robot-interface.l] change default 5 to 1 as :scale in angle-vector
* [robot-intetface.l] check if :controller-type is valid in :angle-vector and :angle-vector-sequence
* [robot-interface.l] Support ctype in :angle-vector-duration
* [robot-interface.l] add :angle-vector-safe for prototype robot
* [robot-interface.l] Add euslisp implementation mannequin mode. (:eus-mannequin-mode)
* [robot-interface.l] modify robot-interface.l to support control_msgs::SingleJointPositionGoal
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Yuki Furuta, Yuto Inagaki

0.1.8 (2015-02-25)
------------------
* Modify wrong maintainer and author name.
* [pr2eus/robot-interface.l] load rosgraph_msgs
* [pr2eus/catkin.cmake] need to call roseus at the end of find_package so that roseus.cmake can read all package files
* Contributors: Kei Okada, Yuto Inagaki

0.1.7 (2015-02-10)
------------------
* [pr2eus] Add sound_play and rosgraph_msgs to find_package to generate messages for roseus
* Updat definition of make-robot-interface-from-name and add
  robot-init-from-name function
* modify :angle-vector-sequence to use angle-vector-duration
* [pr2eus] Add make-robot-interface-from-name function to create
  robot-interface instance from name
* [pr2eus] Repair :angle-vector args document
* return list of t at :wait-interpolation on simulation mode
* fix actionlib error
* fix :wait-interpolation-smooth
* create controller-action-client to process feedback for :wait-interpolation-smooth
* use angle-vector-duration when time is not setted
* add make-plan method for move base
* change variables names.
* enable specification of wait-until-update time for joint-state
* fix: do not use limited buffer for publishing joint state at simulation mode
* add :publish-joint-states-topic keyword to robot-interface for publishing joint_states from the other name
* add :wait t option to speak-en
* add nod function for pr2
* add tuckarm outside
* add test code to check default-robot-interface.l
* add google sound option
* add :move-trajectory-sequence
* add codes in order to use move-trajectory
* avoid to create action and subscriber twice
* reduce assoc
* use let only once
* merge joint-states message which contain other joints. add option to wait until all joint data is updated
* (pr2.l) Generate pr2.l model again
* (`jsk-ros-pkg/jsk_model_tools#18 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/18>`_) pr2eus/make-pr2-model-file.l : remove :camera method which is already committed to irtrobot.l
* do not loop bag file, to privet output TF_OLD_DATA
* add unsubscribe /clock after checking /clock
* Contributors: Hitoshi Kamada, Yuki Furuta, Kei Okada, Yuto Inagaki, JSK Lab member, Chi Wun Au, Masaki Murooka, Ryohei Ueda, Yohei Kakiuchi, Shunichi Nozawa

0.1.6 (2014-05-11)
------------------
* Merge pull request #32 from k-okada/add_roseus_msgs
  remove roseus_msgs from run_depend
* remove roseus_msgs from run_depend

0.1.5 (2014-05-03)
------------------
* Merge pull request #26 from k-okada/22_fix_use_sim_time_check
  fix wrong commit on #22
* fix wrong commit on #22
* Contributors: Kei Okada

0.1.4 (2014-05-02)
------------------
* add roseus_msgs to run_depend
* Contributors: Kei Okada

0.1.3 (2014-05-02)
------------------
* install sample program with executable bit
* Contributors: Kei Okada

0.1.2 (2014-05-01)
------------------
* install only lisp and launch files
* Contributors: Kei Okada

0.1.1 (2014-05-01)
------------------
* add metapackage
* change roseus-svnrevision -> roseus-repo-version, due to https://github.com/jsk-ros-pkg/jsk_roseus/pull/34
* set time-limit 1800
* bugfix: change link name
* disable pr2-ri-test since this requires gazebo
* fix find_package components for groovy, generae missing package via generete-all-msg-srv.sh
* add :controller-timeout keyword to robot-interface to specify
  the timeout to wait controller
* add warn and exit the program for `jsk-ros-pkg/jsk_common#186 <https://github.com/jsk-ros-pkg/jsk_common/issues/186>`_
* Merge pull request `#8 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/8>`_ from YoheiKakiuchi/fix_joint_trajectory
  fix send-trajectory
* `#11 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/11>`_: back to gazebo from gzserver when testing pr2-ri-test.launch
* `#11 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/11>`_: use gzserver instead of gazebo on test
* Merge remote-tracking branch 'origin/master' into youhei-tip
* fix send-trajectory
* fix send-trajectory
* add keyword :joint-states-topic for changing jonit_states name
* install euslisp files in the package root directory: last catkinize commit was also done by murooka
* catkinize pr2eus
* fixed method to get links for new pr2 model
* update pr2 model, fix kinect geometry
* use joint_trajectory_action -> follow_joint_trajectory
* delete commit r5583
* add --no-link-suffix,--no-joint-suffix, concerning backword compatibility
* update pr2 model
* do not use 0.2 sec marge, now the mergin is only 0.1 sec, see https://code.google.com/p/rtm-ros-robotics/issues/detail?id=276 for more detail
* fix window name and draw floor for robot-interface's simulation mode, see Isseue 42, this requries r979(https://sourceforge.net/p/jskeus/code/979/) of jskeus
* add comments for go-velocity arguments and use msec in animation codes
* remove unused local variables
* ignore not existing joint
* add move base range in args of ik
* use :additional-weight-list to set weight without using index of weight vector explicitly ;; test pr2's ik by euscollada/pr2.sh and ik-test.l
* update ros-wait
* fix minor bug
* add :ros-wait method to robot-interface
* fix for using :move-to with /base_footprint as frame_id, [`#234 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/234>`_]
* update parameter for avoiding warning message, [`#233 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/233>`_]
* remove :wait-interpolation finish check on pr2-tuckarm-pose
* move code of visuazlizing trajectory to robot-inreface.l from pr2eus_openrave
* modified loading dependant programs, no longer needed require basic roseus codes
* modified time-limit for low power PC
* add checking correctly finished :wait-interpolation on pr2-tuckarm-pose
* add check code for result of move command, nil will be returned if failed or canceled
* add optional force-stop to :go-stop method
* add check of length c = 2 for dual arm manipulation
* use angle-vector-sequence in angle-vector-with-constraint when ri simulation
* `#216 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/216>`_, support select-target-arm for dual ik
* setup :header :seq, see [`#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_]
* send with move_base_simplw if /move_base/goal failed, see [`#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_]
* use /map frame to send move_base/goal, see [`#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_]
* add description for voice text command
* enable to add arguments for xx-vector methods, which is reported kuroiwa
* r4702 requires fix to make-pr2-model-file.l `#200 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/200>`_
* fix pr2-ri-test to pass the test
* fix :stop-grasp retunrs t
* add :namespace keyword to robot-interface, see [tickets:`#203 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/203>`_]
* remove / from /joint_states according to [tickets:`#202 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/202>`_]
* add -r option (headless) for fuerte
* until hydro, gazebo needs GPU to start, so use DISPLAY to :0.0 for test
* do not wrap around -180/180 degree [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* support :angle-vector over 360 degree, [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* fix time-limit 300->600
* add test code for :angle-vector-with-constraint
* support :arms in :angle-vector-with-constraint, [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* retry twice if :move-gripper is not converged, see [`#159 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/159>`_]
* remove pause mode flag
* add :angle-vector-with-constraiont method, may be we can move to robot-interface?
* add tset code for `#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_
* expand pr2_empty_world.launch files to respawn gazebo
* add test code which show wait-interpolation get dead
* use package:// for loading speak.l
* groovy needs throttled true to launch head-less gazebo?
* add debug message for :start-grasp
* fix `#159 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/159>`_, use robot-update-state to double check the length between tips
* set time-limit to 300
* shorten test code
* return gripper with when simulation mode
* [`#159 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/159>`_] fix start-grasp, resend move-gripper when reached_goal is nil
* add test-start-grasp
* fix commit error [r4499]
* fix: relax camera position differs
* add keyword :use-tf2 and :joint-state-topic to robot-interface
* relax camera position differs
* update pr1012 bag/yaml file for new pr2 robot with sensor robot
* add comment to get bag files
* update pr2.l eus model with sensor head
* update robot_description dump for pr1040
* add PR2_NO argument to make-pr2-model-file-test.launch
* add urdf file which dumped robot_description in pr1040
* add pr2-ri-test.launch
* fix for joint name mismatch between ros and eus
* :move-to retunls nil if not reached to the goal (not closer than 200mm) `#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_
* relax test sequence
* do not use collada_urdf_jsk_patch, use collada_urdf
* (send *ri* :state :worldcoords) return worldcoords when *ri* simulation
* commit add :draw-objects methods, update robot-interface viewer while :move-to in simulation mode
* :move-to takes absolute coordinats as an arguments, currently it does not take into account frame-id, every coords must be relative to world
* add comment
* revert [`#1445 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/1445>`_], since min/max limit of infinite rotational joint has changed from 180 to 270 in https://sourceforge.net/p/jskeus/tickets/25/
* go-pos moves robot in relatively: fix code unless joint-action-enable, Fixed [`#146 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/146>`_]
* fix wreit-r of reset pose from 180->0 [`#145 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/145>`_]
* support :object key in :start-grasp [`#144 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/144>`_]
* support if link-list and move-target is not defined in dual-arm ik mode
* add pr2 ik test with both hands
* support when dual-arm-ik when link-list is not set
* use ros::service-call to change tilt_laser_mux/select [`#94 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/94>`_]
* use check-continuous-joint-move-over-180 for simulation-modep [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* fixed tuckarm-pose angle-vector
* fix: using :{larm,rarm,head,torso}-controller and :{larm,rarm,head,torso}-angle-vector
* add use-tilt-laser-obstacle-cloud
* workaround for unintentional 360 joint rotation problem [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* fix to work pr2-read-state with X-less environment [`#59 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/59>`_]
* change name cancel-all-goals -> go-stop and do not speak in the method, check joint-action-enable, [`#66 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/66>`_]
* add cancel-all-goals
* add test for start-grasp
* add :simulation-modep method to robot-interface
* do not launch viewer when robot-interface is already created [`#71 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/71>`_]
* add pr2-grasp-test
* support no display environment [`#59 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/59>`_]
* fix [`#49 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/49>`_] by mikita
* suport (send *ri* :init :objects (list (roomxxx))) style interface for simulation environment with objects [`#49 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/49>`_]
* fix: add keyword :timeout
* temporary remove :add-controller for pr2
* fix: larm-angle-vector and rarm-angle-vector
* update robot-interface.l for using joint group
* method for adding additional controllers
* fix: tuckarm pose
* add :wait-torso method to pr2-interface
* update for using (send *ri* :potentio-vector)
* fix `#50 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/50>`_, velocity limit for both plug/minus
* added wait option for stop-grasp
* use PLATFORM_FLOAT64 for daeFloat, collada-fom for groovy uses -DCOLLADA_DOM_DAEFLOAT_IS64, update pr2.l to use double precision value
* update: method :state .. use :update-robot-state
* remove debug message
* fix bug for continuous turning
* add a missing variable
* fix: initialization function name should be {robotname}-init
* fix: check absolute rotation angle
* using method :cancel-all-goals instead of :cancel-goal
* add :cancel-angle-vector and :stop-motion method for stopping motion
* add updated urdf file and corresponding bag files
* update pr2 model for fuerte
* autogenerating camera frame for fuerte
* fix calling ros::init if ros is not running
* add :ros-joint-angle for using meter/radian unit
* change: enable to pass robot instance
* fix minor bugs
* fix minor bugs
* fix for liner-joint
* add :send-trajectory to robot interface for using directly JointTrajectory.msg
* move pr2-arm-navigation from pr2eus to pr2eus_armnavigation
* add arm-navigation wrapper for PR2
* add pr2-arm-navigation.l for using arm_navigation stack
* fix go-pos-unsafe, cehck if reached to the original goal using odom and retly if needed, set minimum go-pos-unsafe time to 1000 add debug message
* move kinect_frame transform infrmatin to /opt/ros/electric/urdf/robot.xml
* remove description for static tf nodes
* find vector method from (send self :methods) if exists such as :reference-vector and :error-vector
* find vector method from (send self :methods) if exists such as :reference-vector and :error-vector
* add groupname to slots variables of robot-interface
* add ros node initialize check
* change variable name viewer -> create-viewer
* add pr2-interface setup function
* change for using private queue group in robot-interface in order to divide spin group
* use rosrun rosbag play instaed of rosrun rosbag rosbag
* use equal, not eq to check link name
* use string joint/link name rule, add pr2-senros-robot for camera model
* fix for r3056 (use string as link name too, see `#748 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/748>`_)
* support dual-arm ik which uses target-coords, move-target, and link-list as cons ;; fix move-arm, thre, and rthre definitions
* update tuckarm-pose for non-collision and min-max safe version
* support :joint-action-enable to change real/virtual robot environment. Ask users to really move robot? when :warningp is set, `#758 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/758>`_
* support :stop keyword to :inverse-kinematics
* use lib/llib/unittest.l
* use string-equal to check joint-name
* key of controller action name (:controller -> :controller-action)
* fixed to use string type joint names
* fix for jskeus r773 :gripper method in irtrobot class
* add reference/error vector method in robot-interface
* fix for joint with string name, euscollada/src/collada2eus.cpp@2969
* use string joint-name
* spin once before check robot state variables
* fix typo
* update for `#719 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/719>`_, add accessor to openni camera frames
* support loos checking of cmaera name, currently we are trying to move namer name from string style to keyword style
* use (pr2) to instantiate pr2 robot
* change parent of larm-end-coords from l/r_gripper_parm_link to l/r_gripper_tool_frame
* fix pr2.l compile rule
* use _roscore_failed for not run make-pr2-model-file without roscore and /robot_description environment
* eps=0.01 for camera projection check
* update pr2.l
* update pr2model to r2714 euscollada
* update pr2 model for r2693 or euscollada
* add a test for link weight, update pr2.l model file
* retake pr1012_sensors.bag
* update test bagfile for pr2 sensors and kinect/tf
* check link-coords, currently this is commented out
* fix openni camera link coordinates see jsk_pr2_startup/jsk_pr2_sensors/kinect_head.launch
* update test bagfile for pr2 sensors
* add debug message and add pr2-camera-coords-test
* add debug message
* update pr2eus-test to make robot model on the fly
* update l_finger_tip_link position
* fix syntax error on :publish-joint-state
* fix syntax error on :publish-joint-state
* update publish-joint-state for pr2, publish gripper joint_state
* remove dependency for pr2_* from roseus
* update pr2.l with safty controller limit
* add black color to kinect
* add test for link position
* rename j_robotsound -> robotsound_jp
* sleep 1 second after advertising
* add japanese speech topic for pr2-interface
* move robot-interface from roseus to pr2eus
* added sound_play function
* add kinect camera
* add strict check for camera number test
* fix make-pr2-model-file as urdf_to_collada supports dae file loading
* robot-interface :state with no argument is obsolated, and add warning messages
* :go-pos-unsafe updated, 1000 times msec
* removed initialize-costmap, this is obsolated
* I checked latest pr2.l works well by my program
* pr2-interface :state :odom :pose should return coordinates
* add test for sensor read methods of pr2-interface
* added :set-robot-state1 method to update robot-state variable, and store the time stamp of current joint_states
* changed global frame for (:move-to and :state :worldcoords), /map -> /world
* unchanged min-max angle is OK
* added prosilica and kinect camra to bag in test
* change count for wait slow camera info topic
* do not make error when expected difference between unstable and stable model
* fix assert message type
* add debug messages
* fix tpo in format string
* rename variable, use stable and unstable
* fix camera test code
* fix to work when camera_info is not found
* add make-pr2-model-file-test
* remove debug code
* fix make-pr2-model-file so that other package can use this
* default frame-id of pr2:move-to is /map
* pr2-robot does not calcurate joint-torque in torque-vector method
* changed to use robot-interface
* devide pr2-interface into robot common interface and pr2 specific methods
* check if velocity and efforts in /joint_states are same length as joint list
* added joint-action-enable check for :publish-joint-state
* instantiate transform-listener in ros-interface :init
* error handling when time list contains 0.0 in angle-vector-sequence
* miss understanding of pr2-robot origin coords, base_footprint
* add (if p) in pr2-interface :objects
* fix when frame_id is base_link
* fix compile warning -> velocities in :update-robot-state
* add :state :worldcoords, update :move-to, use :go-velocity after the robot reached gaol using move_base navigation controller
* dissoc before copy-object
* check viewer in :objects, because viewer only exists in simulation mode
* changed go-pos-unsafe to use 80% of max velocity
* remove x::draw-things
* fix :start-grasp, dissoc if already assoced, use x::draw-thing in :objects, etc
* fix segfault
* add :objects for simulation mode to display objects in pr2-interface viewer, also simulation mode is supported in :start-grasp and :stop-grasp
* add :gripper :links to return gripper links
* do not call dynamic reconfigure to static costmap, but it will repaired
* update navigation utility to electric
* add simulation mode to go-pos-unsafe and go-velocity
* add go-pos-unsafe
* update navigation parameter methods in pr2-interface
* change pr2-interface to update robot-model by joint_state msg which contains unknown joint names
* add joint-action-enable for :move-to
* add accessor to :robot and :viewer
* fix when x::*display* is 0
* fix type anlge -> angle
* change :start-grasp :wait nil -> t, and returns the space length of the gripper
* update :move-gripper, move gripper in simulation mode
* update pr2-tuckarm-pose smarter
* fix gripper joint manually
* update tuckarm pose method, and send angle-vector by each controller
* dump euscollada-robot definition to euscollada robot files and update pr2eus/pr2.l
* update pr2.l for latest euscollada/pr2.l ;; use euscollada-robot class instead of robot-model class ;; please refer to jsk-ros-pkg -r1822 commit
* fix previous commit : do not invoke viewer when no x:*display* found
* do not invoke viewer when no x:*display* found
* add pr2-ik-test.l and pr2eus-test.launch
* fix l_gripper_r_finger_tip_link -> l_wrist_roll_link
* add pr2-ik-test.l
* manually fix bug `#560 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/560>`_
* use palm link as parent of endcoords
* update with kinect model
* update pr2 model with safety_limit
* use :state :potentio-vector instead of old :state method call
* update pr2-read-state.l to draw torque
* add max velocity and torque in :init-ending
* set the name of base_trajectory action to same other actions
* fix typo pr2_base_trajectory_action
* update topic name for pr2_base_trajectory_action
* revert accidentally commit
* update namespace of pr2_base_trajectory_action
* add publish-joint-state method, which publish joint_states when joint-action-enable is nil
* set joint-action-enable t before wait-fore pr2-action-server
* wait for joint-velocity to zero, in wait-interpolation for pr2
* add defun make-camera-from-ros-camera-info-aux
* make-camera-from-ros-camera-info-aux is required for non-roseus users
* fix *hrp4* -> robot
* split pr2-interface to pr2-interface and ros-interface
* remove defun make-camera-from-ros-camera-info-aux, which is now defined in roseus-utils.l
* support :state :torque-vector, by mikita
* add effort to state in pr2-interface class
* use :torso_lift_joint method
* add dummy massproperty pr2.l
* add message name to constant in msg definition
* update pr2.l model 2010523
* add clear-costmap, initialize-costmap, change-inflation-range, call clear-costmap when the robot retry move-to function i n (send *ri* :move-to)
* fix contious rotational joint problems, pr2 controller use joint angle value directory, so we add offset before sending the trajectory
* add and fix sub-angle-vector method, fix simulation mode
* :angle-vector-sequence returns angle-vector-sequence
* send only one message in pr2-angle-vector-sequence method
* fix diff-angle-vector in :angle-vector-sequence
* add diff-angle-vector function in :anlge-vector-sequence for calculating velocity vector for interpolation
* cropping angle of infinite rotational joint supported in irtmodel.l
* set :min and :max for infinite rotational joint is *inf* and *-inf*
* add simulation mode code in :angle-vector-sequence
* draw interpolated postures unless joint-action-enable in :angle-vector
* remove typo
* remove spin-once in (:angle-vector-sequence
* remove spin-once in (:angle-vector
* fix :inverse-kinematics move-arm move-target link-list, `#493 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/493>`_
* if no viewer is executed before pr2-interface viewer, set pr2-interface viewer as a defulat *viewer*, so that users are able to use them as a default view
* fix fingertip pressure zero-reset, update pr2-read-state sample
* add ** to msg constant type
* we can send JointTrajectoryActionGoal to torso and head in diamondback
* update grasp timing in tuckarm-pose, add pr2-reset-pose
* add pr2 tuckarm pose function
* remove useless number 1 in ros::ros-warn
* use ros::ros-warn instaed of warning-message
* support sending go-velocity countinously, and once
* support sending go-velocity countinously
* fix go-velocity function
* add go-velocity method using trajectoy and safe_teleop
* add go-velocity to pr2-interface.l
* torso and head did not accept time_from_start, it only accept duration
* update pr2.l with :camera and :cameras
* add to generate :cameras and :camera by chen and k-okada
* require pr2-utils, show viewer in NON-joint-action-enable mode
* if robot-joint-disabled, :state sends recieved angle-vector
* pr2-interface :init works unless it connected to pr2
* update ros-infro comment
* update pr2.l using r769
* update :*-cmaera method definitoin, support forward-message-to
* fix :inverse-kinematics with use-base
* update :inverse-kinematics with use-base
* update :inverse-kinematics support use-torso, use-base, move-arm
* In head point action, pointing_frame is not used, and change translate length
* add fingertip pressure subscriber, to use finger-pressure call reset-fingertip beforehand
* set time out for gripper action
* action start time should be future, i think
* use :wait-interpolation, remove sleep
* fix do not generate pr2.l if it already exists
* add move_base_msgs
* fix problem, when not add roseus to /home/k-okada/ros/cturtle/ros/bin:/usr/local/cuda/bin/:.:/home/k-okada/bin:/usr/local/bin:/usr/local/svs/bin:/usr/java/j2sdk1.4.1/bin/:/usr/bin:/bin/:/usr/sbin:/sbin:/usr/X11R6/bin:/usr/local/jsk/bin:/home/k-okada/ros/cturtle/jsk-ros-pkg/euslisp/jskeus/eus/Linux/bin:/bin:/usr/h8300-hitachi-hms/bin:/usr/local/ELDK4.1/usr/bin:/home/k-okada/prog/scripts:/usr/local/src/gxp
* rename cmaera->camera-model, viewing->vwing
* update pr2model with new make-camera-from-ros-info-aux
* update to new make-camera-from-ros-info-aux
* update pr2 model file
* add pr2 model file at 100929
* delete load-pr2-file.l
* load-pr2-file is removed, now we use make-pr2-modle-file
* generate pr2model from camera_info and /robot_description
* front of high_def_frame is +x
* set pointing_frame to look-at-point action goal
* fix to move head-end-coords in sending current pose
* update :angle-vector-sequence to work with real-pr2 robot
* add :angle-vector-sequence based on interpolator::push in rats/src/interpolator.cpp
* update :send-pr2-controller interface (:send-pr2-controller nil (action joint-names all-positions all-velocities starttiem duration)
* support send *pr2* :inverse-kinematics c
* add test code for load-pr2-file
* add load-pr2-file
* add dual arm jacobian, torque sample by s.nozawa
* fix pr2 gripper action sending
* add hrp2 compatible :go-pos [m] [m] [degree] method
* remove waiting for move-base action in pr2-interface :init
* change to startable pr2-interface when move_base not found
* add :move-to method and move-base-action slot variable
* add :gripper and :override :limb of irtrobot.l to suppoer send *pr2* :larm :gripper :angle-vector
* change to use roseus, whcih automatically load roseus.l eustf.l actionlib.l
* change to use pr2.l in pr2eus directory
* rosmake pr2eus to generate pr2.l
* fix to use require for eustf and actionlib
* revert to r527 float mod is supported in eus
* result of (r2deg p) should be integer for using mod
* crop joint-angle to +- 360 in :state :potentio-vector
* add depend package
* add gripper action to pr2-interface
* wait at most 10 seconds
* fix return-from, in :state method
* fix syntax error (require :keyword path) <- (require path)
* add pr2_controllers_msgs
* fix to use package:// load style
* rename roseus-add-{msgs,srvs}->ros::roseus->add-{msgs,srvs}
* pr2model is obsoluted
* add pr2 ros controlelr and euslisp interface
* add utility functions for pr2 euslisp model
* add sample program and launch file for PR2 users
* remove piped-fork and use ros::rospack-find
* modify pr2model.l to head joint
* add reset manip pose to pr2
* fix pr2model, support :fix and :relative mode in :inverse-kinematics, see hold-cup in 2010_05_pr2ws/sample-motion.l for example
* override :init, set reset-pose as initial pose
* fix many bags to move pr2 by joint angle actionlib interface
* change middle-body-joint-angle-list API: omit string-upcase for joitn name
* add pr2eus model, which depends on urdf2eus
* Contributors: Haseru Chen, Yuki Furuta, Kei Okada, Yuto Inagaki, Satoshi Iwaishi, Manabu Saito, Shunichi Nozawa, Kazuto Murase, Masaki Murooka, Ryohei Ueda, Yohei Kakiuchi, Yusuke Furuta, Hiroyuki Mikita, Otsubo Satoshi
