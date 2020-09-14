^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eusurdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2018-12-05)
------------------
* Fix for being deprecated tag: cfmDamping -> implicitSpringDamper (`#222 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/222>`_)
* [eusurdf] add euslisp to INSTALL_DIRS (`#220 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/220>`_)
* Contributors: Juntaro Tamura, Kentaro Wada

0.4.2 (2018-09-04)
------------------

0.4.1 (2018-08-01)
------------------

0.4.0 (2018-07-25)
------------------
* [eusurdf] add cmake target for generating Semantic map from eusmodel (`#194 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/194>`_)
  * [eusurdf] add cmake target for generating ontology if generator exists
  * [eusurdf][package.xml] add python-lxml as run_depend

* Fix for arm processor (`#209 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/209>`_)
  * use LinuxARM ARCHDIR for CMKAE_SYSTEM_PROCESSOR armv/aarch64

* Contributors: Yohei Kakiuchi, Yuki Furuta

0.3.5 (2017-02-23)
------------------
* add gazebo_ros to build_depend
* Contributors: Kei Okada

0.3.4 (2017-02-22)
------------------
* add python-lxml
* Contributors: Kei Okada

0.3.3 (2017-02-18)
------------------

0.3.2 (2017-02-18)
------------------

0.3.1 (2017-02-18)
------------------
* for kinetic release (`#197 <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/197>`_)
  * eusurdf/euslisp/eusmodel_to_urdf.l: if collada_urdf_jsk_patch does not exists, use collada_urdf
  * CMakeLists.txt, cmake/eusurdf.cmake : use collada_urdf if collada_urdf_jsk_patch is not found
* Contributors: Kei Okada

0.3.0 (2017-01-16)
------------------
* [eusurdf] fix: use :if-exists :new-version instead of :overwrite  (`#196  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/196>`_)

* use package://eusurdf instead of model:// (`#195  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/195>`_)
  * [eusurdf] fix: try catch
  * [eusurdf] use 1 launch file to spawn; add option to publish tf for model
  * [eusurdf][convert-eus-to-urdf.l] fix: add / for directory path
  * [eusurdf][textured_models] use package://eusurdf instead of model:// for xacro
  * [eusurdf][urdf_to_xacro.py] convert model:// -> package://eusurdf

* [eusurdf] generate urdf.xacro for model / scene (`#193  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/193>`_)
  * [eusurdf][package.xml] add python-lxml as run_depend
  * [eusurdf][README.md] append usage about spawning urdf
  * [eusurdf][textured_models] add urdf.xacro for textured_models
  * [eusurdf] generate urdf.xacro from eusmodel / eusscene to make unified urdf

* parallelize model generation (`#188  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/188>`_)
  * [eusurdf][convert-eus-to-urdf.l] remove tmp-dir-no-duplication
  * [eusurdf][eusscene_to_world.l] raise error when model name is undefined
  * [eusurdf][convert-eus-to-urdf.l] fix: forgot to update
  * [eusurdf][convert-eus-to-urdf-test.l] update test
  * [eusurdf][convert-eus-to-urdf.l] create directory for world file if not created
  * [eusurdf][convert-eus-to-urdf.l] make argument world-file-path necessary; remove random-seed
  * [eusurdf] get collada_to_urdf bin path from cmake
  * [eusurdf] avoid intermediate files permission issue
  * [eusurdf][convert-eus-to-urdf.l] keep back compatibility
  * [eusurdf] cleanup old stuff
  * [eusurdf][textured_models] drop suffix for models
  * [eusurdf] parallelize model generation

* Contributors: Yuki Furuta

0.2.5 (2016-10-18)
------------------
* [eusurdf] add test of model conversion (`#181  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/181>`_)
  * [eusurdf/CMakeLists.txt,package.xml] test model conversion in eusurdf package.
  * [eusurdf/test] add test directory in eusurdf. add script and launch to test model conversion.
  * [eusurdf/package.xml] add maintainer of eusurdf package.
  * [eusurdf/euslisp/convert-eus-to-urdf.l] add documentation of euslisp function to convert irteus->urdf.

* [eusurdf/euslisp/convert-eus-to-urdf.l] get name from converted eus model if the argument name is not passed. (`#169  <https://github.com/jsk-ros-pkg/jsk_model_tools/pull/169>`_)

* Contributors: Masaki Murooka

0.2.4 (2016-04-30)
------------------
* Ignore world files
* Contributors: Kentaro Wada

0.2.3 (2015-12-31)
------------------
* [eusurdf/textured_models] add iemon/hamburger/wanda to texture_models
* Contributors: Yuki Furuta

0.2.2 (2015-12-11)
------------------
* [textured_models/room73b2-hitachi-fiesta-refrigerator-0] transparent inside board of fridge
* [textured_models/room73b2-georgia-emerald-mountain-0] add texture of georgia can
* [textured_models/room73b2-hitachi-fiesta-refrigerator-0/model.urdf] update to more realistic physics parameters
* [textured_models] add georgia can
* [CMakeLists.txt] install eusurdf/worlds directory
* [CMakeLists.txt] add roseus to run_depend. use environment variable to get eusdir. call roseus without rosrun.
* [eusurdf] add textured_models/
* Contributors: Yuki Furuta, Masaki Murooka

0.2.1 (2015-11-27)
------------------
* package.xml; we nee do rosrun to generate models
* generate eus model and convert it to urdf from voxel grid
* use the resolved path of ros package to find eusurdf directory when path is nil. You can pass eusurdf path as argument to run in catkin build.
* add generate-model-from-voxel.l to generate eus cube list from voxel.
* Contributors: Kei Okada, Masaki Murooka

0.2.0 (2015-11-26)
------------------
* move to 0.2.0, which only available from indigo

* add instruction of converting eus->urdf
* modify sample usage comment of irteus2urdf-for-gazebo.
* use the resolved path of ros package to find eusurdf directory when path is nil. You can pass eusurdf path as argument to run in catkin build.
* add .gitignore to keep model directory
* generate model directory if not found.
* delete manifest.xml for gazebo model directory.
* Contributors: Masaki Murooka

0.1.13 (2015-09-01)
-------------------
* [eusurdf/package.xml] export gazebo_model_path for gazebo_ros
* - [eusurdf] remove rosbuild related scripts
  revert travis
* generate random tmp directory to avoid overwrite
* fix to use no rospack find nor rosrun for eusurdf
* convert models when catkin build
* add files to convert irtmodel to urdf
* delete converted urdf models in models directory.
* Contributors: Yuki Furuta, Masaki Murooka

0.1.12 (2015-05-07)
-------------------

0.1.11 (2015-04-09)
-------------------

0.1.10 (2015-04-02)
-------------------

0.1.9 (2015-04-01)
------------------

0.1.8 (2015-01-07)
------------------

0.1.7 (2014-12-19)
------------------
* Move scripts to euscollada to avoid catkinization of eusurdf
* Use link name, not joint name as parent link, but the solution is adhock
* add addLink function to add_sensor_to_urdf.py
* Add script to add end effector frames to urdf from yaml file for euslisp
* Add script to add sensor (fixed link) to urdf
* added moveit scene files
* add urdf models to eusurdf/models.
* Contributors: Ryohei Ueda, Masaki Murooka

0.1.6 (2014-06-30)
------------------

0.1.5 (2014-06-29)
------------------

0.1.4 (2014-06-15)
------------------

0.1.3 (2014-05-01 17:24)
------------------------

0.1.2 (2014-05-01 09:31)
------------------------

0.1.1 (2014-05-01 01:25)
------------------------
* set eusurdf and euslisp_model_conversion_tester to ROS_NOBUILD
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: make model directory before running xacro when building eusurdf
* fix hoge.stl->model.stl to pass hoge/fuga check
* update dirctory for xml2sxml
* use face-to-triangle-aux for triangulate faces
* update for using simple conversion
* fix for using package:// at inside jsk
* fix for using package:// at inside jsk
* fix, if link has no mesh
* fix checking which link has glvertices
* remove jsk internal dependancy
* add code for parsing inertial parameter
* debug for using fixed joint
* update for parsing sdf
* fix error message
* add heightmap tag to geometry/visual
* update for using :translate-vertices in eusurdf.l
* update for parsing cylinder and plane geometry
* update for using multi visual/geometry tags in link
* fix for parsing sdf file
* add eusurdf (copy from jsk-ros-pkg-unreleased)
* Contributors: Kei Okada, Ryohei Ueda, nozawa, youhei
