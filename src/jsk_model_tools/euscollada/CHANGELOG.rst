^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package euscollada
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2018-12-05)
------------------
* fix for urdfmodel 1.0.0 (melodic) (`#221 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/221>`_)
* Contributors: Kei Okada

0.4.2 (2018-09-04)
------------------
* collada2eus_urdfmodel: append :provide on bottom; force flush (`#219 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/219>`_)
* Contributors: Yuki Furuta

0.4.1 (2018-08-01)
------------------
* [euscollada] fix collision model for version 0.4.0f( `#218 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/218>`_)
* Contributors: Yohei Kakiuchi

0.4.0 (2018-07-25)
------------------
* Replace euscollada for using ROS collada/urdf parser. It can convert both collada and urdf to eusmodel (`#216 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/216>`_)
  * [euscollada] previous collada2eus can be used by collada2eus.orig
  * [euscollada, collada2eus_urdfmodel] switch default model format (:collada->:urdf)
  * [euscollada] use collada2eus_urdfmodel as collada2eus
  * [euscollada,collada2eus_urdfmodel] fix parsing arguments
  * [euscollada,collada2eus_urdfmodel] fix reset-pose
  * [euscollada,collada2eus,collada2eus_urdfmodel] fix centroid
  * [euscollada, euscollada-robot.l] fix inertia conversion when weight is zero
  * [euscollada] fix message when robot.yaml has shorter length of angle-vector
  * [euscollada] do not make small cube for link without geometry
  * [euscollada] fix using _fixed_jt as slot name for fixed joint
  * [euscollada] add add_normal argument

* [euscollada] fix travis testtest (`#215 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/215>`_)
  * fix timeout for reading stream
  * set origin of robot before comparing centroid

* Update collada2eus_urdfmodel (`#212 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/212>`_)
  * [euscollada/collada2eus_urdfmodel] update for using non mesh geometry(BOX, CYLINDER, SPHERE)
  * [euscollada/collada2eus_urdfmodel] fix, the same as `#174 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/174>`_
  * [euscollada/collada2eus_urdfmodel] fix warning message for reading urdf file
  * [euscollada/collada2eus_urdfmodel] fix min-max range for a continuous joint
  * [euscollada/collada2eus_urdfmodel] fix for writing non mesh geometry

* fix to run installed euscollada (`#213 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/213>`_)
  * enable to run within write-protected directory
  * euscollada: install sh fails as PROGRAMS

* src/collada2eus.cpp: fix wrong python style string `#174 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/174>`_ (`#204 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/204>`_)

* Contributors: Kei Okada, Yohei Kakiuchi

0.3.5 (2017-02-23)
------------------

0.3.4 (2017-02-22)
------------------

0.3.3 (2017-02-18)
------------------
* urdfdom is for hydro, now we can drop (forget to remove run_depends)
* Contributors: Kei Okada

0.3.2 (2017-02-18)
------------------
* urdfdom is for hydro, now we can drop
* Contributors: Kei Okada

0.3.1 (2017-02-18)
------------------
* for kinetic release (`#197 <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/197>`_)
  * collada2eus : convertnig const char* makes strange behavior (il->second becomes NONE)
* Contributors: Kei Okada

0.3.0 (2017-01-16)
------------------
* Fix conversion of mass properties (`#192  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/192>`_)
  * [euscollada] fix test (samplerobot centroid and pose)
  * [collada2eus] fix conversion of mass properties, when there are links which are not member of links
  * [euscollda] add tests
* Contributors: Yohei Kakiuchi

0.2.5 (2016-10-18)
------------------
* [collada2eus] fix closing /dev/null (`#186  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/186>`_)
* Add documentation for predefined poses (`#174  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/174>`_)
* Contributors: Kentaro Wada, Yohei Kakiuchi

0.2.4 (2016-04-30)
------------------
* Support building on OS X
* Support passing limbs to predefined pose method
  Modified:
  - euscollada/src/collada2eus.cpp
* [euscollada/src/collada2eus_urdfmodel.cpp] Fix location of gl::vertices setting. Move after geom is set.
* Contributors: Kentaro Wada, Shunichi Nozawa

0.2.3 (2015-12-31)
------------------

0.2.2 (2015-12-11)
------------------

0.2.1 (2015-11-27)
------------------

0.2.0 (2015-11-26)
------------------
* move to 0.2.0, which only available from indigo

* keep backward compatibility, add glvertices slot to collada-body
* [euscollada/src/euscollada-robot.l] use-6dof-joint-leg-gain nil by default
* make qpqmodel with glvertices
* merge euscollada-robot.l
* fix large matrix
* [euscollada/src/euscollada-robot_urdfmodel.l, euscollada-robot.l] Add arugment to use 6dof-joint leg weighing for evaluate the influence with it and without it.
* [euscollada] Compare rosversion using the alphabetical order
* fix typo
* use changes for using glbody in irtgl.l
* Contributors: Kentaro Wada, Shunichi Nozawa, Yohei Kakiuchi

0.1.13 (2015-09-01)
-------------------
* return if key not found
* fix for yaml file without sensors
* [euscollada] Fix replace_xmls syntax in add_sensor_to_collada.py
  1. Force to use string. yaml parser automatically parse digit numbers as
  integer or float. OTH, minidom parser always outputs everything in
  string.
  We force to convert yaml parser's output into string value.
  2. Raise exception if there is no tag section.
  3. Do not remove parent node if replaced_attribute_value syntax is used
* [euscollada] Support xml force-replacing in add_sensor_to_collada.py
* fix reading texture coords
* [euscollada/src/euscollada-robot*.l] Always make pqpmodel for detailed shape according to https://github.com/euslisp/jskeus/pull/232
* 0.1.12
* update CHANGELOG (For releasing 0.1.12 DRC Final version)
* [src/collada2eus.cpp] on newer yaml, doc["angle-vector"]["reset-pose"] did not raise error
* [jsk_model_tools] remove old rosmake files
* [collada2eus.cpp] do not exit when polylistElementCound or polygoneElementCount is 0
* [euscollada/src/collada2eus.cpp] super ugry hack untilyaml-cpp 0.5.2
* [collada2eus] set verbose=true when --verbose
* [euscollada] Removed unnecessary fprintf in collada2eus.cpp
* [euscollada] Add size check to end-coords translation/rotation because undefiend limb end-coords transformation/rotation breaks matching of parentheses in yaml-cpp 0.5.
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Iori Kumagai, Iori Yanokura

0.1.12 (2015-05-07)
-------------------
* [src/collada2eus.cpp] on newer yaml, doc["angle-vector"]["reset-pose"] did not raise error
* [jsk_model_tools] remove old rosmake files
* [collada2eus.cpp] do not exit when polylistElementCound or polygoneElementCount is 0
* [euscollada/src/collada2eus.cpp] super ugry hack untilyaml-cpp 0.5.2
* [collada2eus] set verbose=true when --verbose
* [euscollada] Removed unnecessary fprintf in collada2eus.cpp
* [euscollada] Add size check to end-coords translation/rotation because undefiend limb end-coords transformation/rotation breaks matching of parentheses in yaml-cpp 0.5.
* Contributors: Kei Okada, Iori Kumagai

0.1.11 (2015-04-09)
-------------------
* [euscollada] Suppress output of debug information from collada2eus
  and add --verbose option to print the debug information
* Contributors: Ryohei Ueda

0.1.10 (2015-04-02)
-------------------
* [euscollada] install src/ scripts/
* Contributors: Kei Okada

0.1.9 (2015-04-01)
------------------
* [euscollada] Update urdf_patch.py to handle joint without xyz and rpy tag and to output patched urdf to standard output
* [euscollada] Support multiple links in remove_sensor_from_urdf.py
* [euscollada] Remove pyc file added by mistake
* [euscollada] (remove_sensor_from_urdf.py) Add script to remove link from urdf
* [esucollada] update parseColladaBase.py and add_sensor_to_collada.py for handling urdf file
* Contributors: Ryohei Ueda, Yohei Kakiuchi

0.1.8 (2015-01-07)
------------------

0.1.7 (2014-12-19)
------------------
* fix parsing sensors from yaml file, sensor_id should be optional
* Get sensor id from sid of sensor tag and sort euslisp sensors by sensor's sid
* Script to compute difference of two urdfs and dump it to yaml file, and apply the yaml file to urdf file as a patch
* add camera model
* Move scripts to euscollada to avoid catkinization of eusurdf
* add sensor coordinates to eus model while converting from urdf model
* add code for viewing convex bodies
* fix order of qhull vertices
* use multiple visual
* update add_sensor_to_collada.py for adding sensor from yaml file
* Merge remote-tracking branch 'origin/master' into use_loadable
* update for compiling on indigo, use liburdfdom and can use yaml-cpp-0.5
* add use_loadable
* fix for using fixed_joint
* fix inertia frame
* remove nan in normal
* (collada2eus.cpp) : Parse multiple translate and rotate tag for sensor definition
* Contributors: Ryohei Ueda, Yohei Kakiuchi, Shunichi Nozawa

0.1.6 (2014-06-30)
------------------
* package.xml: add collada_urdf to run_depend and build_depend
* Contributors: Kei Okada

0.1.5 (2014-06-29)
------------------
* catkin.cmake: add *.yaml and *.sh to install
* pr2.sh: Support Hydro pr2 model path
* Contributors: Kei Okada, Shunichi Nozawa

0.1.4 (2014-06-15)
------------------
* revert codes for collision model making according to https://github.com/euslisp/jskeus/pull/93 and https://github.com/jsk-ros-pkg/jsk_model_tools/pull/46
* Enable euscollada conversion test ;; Add dependency on pr2_mechanism_model to travis.yaml ;; Fix cmake and use unittest.l in pr2.sh to trap Euslisp error
* (https://github.com/jsk-ros-pkg/jsk_model_tools/issues/18) euscollada/src/collada2eus_urdfmodel.cpp : do not overwrite sensor methods
* (jsk-ros-pkg/jsk_model_tools/issues/18) euscollada/src/collada2eus.cpp : do not overwrite sensors methods ;; sensors method are supported from euslisp/jskeus/pull/92
* (jsk-ros-pkg/jsk_model_tools/issues/41) euscollada/src/euscollada-robot*.l : move collision model codes to irtrobot.l https://github.com/euslisp/jskeus/pull/93
* (jsk-ros-pkg/jsk_model_tools/issues/18) euscollada/src/euscollada*.l : remove deprecate sensor methods ;; latest sensor methods are added and testes by https://github.com/euslisp/jskeus/pull/92
* fix sensor coords
* Contributors: Yohei Kakiuchi, Shunichi Nozawa

0.1.3 (2014-05-01)
------------------
* Merge pull request `#35 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/35>`_ from k-okada/add_tf_depends
  add tf to depend
* Contributors: Kei Okada

0.1.2 (2014-05-01)
------------------
* put catkin_package after find_package(catkin)
* Contributors: Kei Okada

0.1.1 (2014-05-01)
------------------
* check if pr2_mechanism_model exists
* add rosboost_cfg, qhull and cmake_modules to depends
* use assimp_devel pkgconfig
* (euscollada) update for assimp_devel in jsk_common (`#20 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/20>`_)
* support string argument for sensor accessor methods discussed in https://github.com/jsk-ros-pkg/jsk_model_tools/issues/18
* add rosbduil/mk to depend
* remove denepends to jsk_tools whcih is used for launch doc
* add add_dependancies
* remove urdf_parser, it is included in urdfdom
* add making collada2eus_urdfmodel in catkin
* udpate euscollada for groovy
* update manifest at euscollada
* remove debug message
* fix make pr2 instance if *pr2* does not exists
* do not use glvertices on collada-body if it does not exists
* fix using non-existing tag/body
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: omit ik demo
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: omit PR2 IK test from euscollada to avoid intermediate dependency
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: add yaml-cpp to euscollada dependency
* sorting sensor order of urdfmodel
* add small cube if geometry does not exist
* add comment for using assimp_devel
* add some scripts for fixing collada error
* add printing sensor methods to euscollada_urdf
* add euscollada-robot_urdfmodel.l
* revert euscollada-robot.l
* update mesh post process
* fix minor bug
* update collada2eus_urdfmodel
* install src directory in euscollada because euscollada-robot.l is in src
* install collada2eus
* fix link association and material on collada2eus_urdfmodel.cpp
* update collada2eus_urdfmodel.cpp
* update collada2eus_urdfmodel.cpp
* add rosdep collada_urdf for rosdep install
* update collada2eus_urdfmodel.cpp
* change description in euscollada-robot.l
* small update
* remove compile test program
* add dependancy for assimp
* add collada2eus_urdfmodel, but it is not working well now
* add collada2eus for using urdfmodel
* dump sensor name as string instead of using symbol with colon to keep lower-case and upper-case
* add writeNodeMassFrames function ;; write node MassFrame regardless of geometory existence
* fix parenthesis of bodyset-link definition ;; separate mass frame writing
* find thisArticulated which has extra array
* append additional-weight-list
* use additional-weight-list instead of weight
* separate defining of sensor name method
* catkinze euscollada
* fix bug discussioned in [`#243 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/243>`_]
* add for reading <actuator> <nominal_torque>
* add :max-joint-torque
* move collada-body definition to euscollada-robot.l
* add checking body has glvertices
* fix typo in :init-ending
* add make-detail-collision-model-from-glvertices-for-one-link
* use transform from associated parent link
* add name to end-coords
* enable to generate and display models which bodies have no vertices
* fix - -> _ for bodies name
* add robot_name to link body
* use :links to obtain sensor's parent link
* create output(lisp) file after successfully parsed collada file, see https://code.google.com/p/rtm-ros-robotics/issues/detail?id=164
* add use_speed_limit parameter to collada2eus for avoiding to use speed-limit
* fix matrix multiple bug for inertia tensor, [`#222 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/222>`_]
* modify precision for printing euslisp model file, [`#222 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/222>`_]
* add target for conversion from irteus to collada ;; does not add this conversion to default ALL target
* use collad_directory for irteus -> collada output directory
* remove test code depends on glc-capture
* add barrett-wam and debug message
* add barrett test
* comment out warning message
* do not support non-sensor keyword method
* link's instance name have _lk suffix, buf link's name itself does not have suffix, [`#200 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/200>`_]
* update: nao.sh
* fix: joint-angle on nao.yaml
* add add_joint_suffix and set add_link_suffix and add_joint_suffix as default
* add accessor by limb name
* fix :set-color method of collada-body
* add dump of imu sensor and imusensor methods
* add :set-color method for overwrighting geometry color
* add --add-link-suffix option to collada2eus for avoiding to add the same name to link and joint
* move collada2eus_dev.cpp to collada2eus.cpp
* move collada2eus.cpp to collada2eus_old.cpp
* fix: parsing transformation in conllada file (experimental)
* revert [`#1445 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/1445>`_], since min/max limit of infinite rotational joint has changed from 180 to 270 in https://sourceforge.net/p/jskeus/tickets/25/
* set recommended stop and cog-gain param
* overwrite fullbody-inverse-kinematics method ;; test on euscollada-robot
* switch collada2eus to use glvertices for visualization
* fix wreit-r of reset pose from 180->0 [`#145 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/145>`_]
* add dom like function to using sxml
* update index.rst,conf.py by Jenkins
* update index.rst,conf.py by Jenkins
* update index.rst,conf.py by Jenkins
* use collada_urdf instead of collada_urdf_jsk_patch, jsk_patch is subitted to upstream see https://github.com/ros/robot_model/pull/15/
* update index.rst,conf.py by Jenkins
* update index.rst,conf.py by Jenkins
* merge updates on collada2eus.cpp
* merge updates on collada2eus.cpp
* remove unused string
* find root-link by tracing limb's link list
* use robot_name instead of thisNode->getName
* add robotname to body classes to avoid duplicate naming
* add comment for mass property fix ;; add sensor calling method according to pr2eus/pr2.l's :camera method
* add getSensorType for attach_sensor
* add force-sensors from attached sensor according to pr2eus/pr2.l's :cameras method
* add attach_sensor coords method
* fix bug of mass_frame interpretation ;; support multiple mass_frame description (e.g., VRML->collada file) ;; tempolariry calculate link-local mass property in euscollada-robot's :init-ending
* fix for converting multiple meshe groups
* add collada2eus_dev for development version using glvertices
* fix bug in manipulator's make-coords ;; :axis must non-zero vector ;; some codes about :axis should be fixed
* fix for groovy
* fix for groovy, not using new DAE()
* move rosdep from euscollada to jsk_model_tools since due to package euscollada being in a satck
* update index.rst,conf.py by Jenkins
* add eus_assimp for eusing assimp library on EusLisp
* move euscollada,collada_tools,assimp_devl to jsk_model_tools
* Contributors: Ryohei Ueda, Yohei Kakiuchi, Kei Okada, Shunnichi Nozawa, Masaki Murooka
