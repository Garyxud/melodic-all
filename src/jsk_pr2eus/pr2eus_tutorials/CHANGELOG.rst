^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2eus_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.14 (2019-02-11)
-------------------
* add jsk_interactive_marker (`#349 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/349>`_ )
  c.f. https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/348
* Contributors: Kei Okada

0.3.13 (2017-07-14)
-------------------

0.3.12 (2017-07-11)
-------------------

0.3.11 (2017-06-25)
-------------------
* add test code for pr2 dual arm IK (`#272 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/272>`_ )
* Contributors: Chi Wun Au

0.3.10 (2017-03-02)
-------------------

0.3.9 (2017-02-22)
------------------

0.3.8 (2017-02-07)
------------------

0.3.7 (2016-11-08)
------------------

0.3.6 (2016-11-02)
------------------

0.3.5 (2016-09-16)
------------------

0.3.4 (2016-06-22)
------------------

0.3.3 (2016-05-28)
------------------

0.3.2 (2016-05-26)
------------------

0.3.1 (2016-05-22)
------------------
* [pr2eus_tutorials/package.xml] add roseus_tutorials to run_depend
* [pr2eus_tutorials/launch/pr2_tabletop.launch] fix camera namespace openni -> kinect_head_c2
* [pr2eus_tutorials/launch/pr2_tabletop.launch] fix rviz config path
* [pr2eus_tutorials] cleanup directory
  - move gazebo related launch files to `launch/gazebo` directory
  - move rviz config file to `config` directory
  - fix include path in launch files
* Contributors: Yuki Furuta

0.3.0 (2016-03-20)
------------------
* add files for reach object demo.
* [pr2eus_tutorials] remove dependency to pr2_gazebo.
* catkinize pr2eus_tutorials
* Contributors: Kei Okada, Masaki Murooka

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

0.1.7 (2015-02-10)
------------------
* add empty map in pr2 gazebo startup
* add startup launch file for gazebo pr2
* rename parameter and delete unused arg
* changed pos_z
* changed codes to be used in hydro
* add rot\_{rpy} arguments to spawn_cylinder.launch
* add object spawner for gazebo
* remove absolute path
* Contributors: YoheiKakiuchi, ohara, tarukosu

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
* fix tabletop launcher
* update some template_grasp files
* fixed launch file path by addint _frontend.
* update parameter at base_trajectory
* update pr2_gazebo_empty.launch on pr2eus_tutorials
* fix typo null -> nil
* comment out dependancy unreleased package
* update tabletop-sample.l
* add comment to tabletop-sample.l
* update files for template grasp
* fix using arm navigation overridingopenrave
* temporary backup for migration
* Added some general comments. Also removed some old commented code that is not used.
* add one shot subscriber for point cloud
* fix bug
* update proc-detection
* add subscribe-detection-result.l
* add launch_objectdetection arguments for publish /ObjectDetection
* remove magic number. Using approach distance
* fixed the bug of unnecessary translation of graps-cds
* fix: arm-navigation sample
* fix: remove magic number
* add parameter: convert_to_base_link
* added hand-coords visualizer for image_view2
* added place function, check colliderreset
* launch realtime_tabletop on default template_grasp launch
* added y-or-n-from-tablet function
* bugfix coodinate transform of grasp pose
* add pr2_template_grasp_sim.launch
* update and add launch_object_manipulation
* merged comments and update from obsolates
* add comments
* added template_grasp samples, pick from android
* add comment
* fix: topic name for real robot
* add pr2_tabletop.launch
* add argument nav:=true to interactive_manipulation
* fix: launch_rviz -> run_rviz
* add goto-init-pose
* add pr2_tabletop_sim.launch
* fix: typo
* add pr2_interactive_manipulation_sim.launch
* add objects to pr2_gazebo_objects.launch
* add pr2eus_tutorials for using pr2eus software on simulation environment
* Contributors: YoheiKakiuchi, chen, kazuto, tatu, y-tnaka, youhei
