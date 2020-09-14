^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ml_classifiers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2019-04-15)
------------------
* Merge pull request `#3 <https://github.com/astuff/ml_classifiers/issues/3>`_ from dirk-thomas/patch-1
* add build dep on ros_environment to ensure the env variable ROS_VERSION is defined
* Contributors: Dirk Thomas, Joshua Whitley

1.0.0 (2019-04-09)
------------------
* Merge pull request `#2 <https://github.com/astuff/ml_classifiers/issues/2>`_ from astuff/ros-transition
  ROS2 Hybrid Package - now builds in ROS Kinetic/Melodic and ROS2 Crystal
* ROS2: Making classifier_server a class.
* Creating separate ros1/ros2 xml files.
* ROS1: Removing roslint in favor of ROS2 linting.
* ROS2: Disabling boost in pluginlib.
* ROS2: Updating copyrights to conform to ament_copyright.
* ROS2: Fixing cmake exports.
* Contributors: Joshua Whitley

0.4.1 (2019-03-21)
------------------
* Replacing typedefs with aliases.
* Add run_tests hook for roslint.
* Adding C++11 specifier.
* Contributors: Joshua Whitley

0.4.0 (2019-02-26)
------------------
* Updating URLs in package.xml.
* Updating README and package.xml with new data.
* Merge pull request `#1 <https://github.com/astuff/ml_classifiers/issues/1>`_ from sniekum/master
  Merging from upstream before taking ownership.
* CI: Adding CircleCI tests.
* Replacing createClassInstance with createInstance in pluginlib.
* Merge pull request `#7 <https://github.com/astuff/ml_classifiers/issues/7>`_ from astuff/melodic-devel
  Melodic fixes.
* Fixed compilation problems in Melodic.
* Fixing CMakeLists.txt and updating package.xml to version 2.
* Merge pull request `#6 <https://github.com/astuff/ml_classifiers/issues/6>`_ from wkentaro/migration-to-jade
  [ml_classifers] eigen -> Eigen3 in CMakeLists.txt
  See: http://wiki.ros.org/jade/Migration#Eigen_CMake_Module_in_cmake_modules
* Merge pull request `#3 <https://github.com/astuff/ml_classifiers/issues/3>`_ from jolting/indigo-devel
  Fix build for Indigo
* Contributors: Joshua Whitley, Kentaro Wada, Scott Niekum

0.3.1 (2014-09-15)
------------------
* Fix build for indigo
* Merge pull request `#2 <https://github.com/sniekum/ml_classifiers/issues/2>`_ from davetcoleman/rospack_dep
  Missing dep on rosdep
* Missing dep on rosdep
* updated version number
* Merge pull request `#1 <https://github.com/sniekum/ml_classifiers/issues/1>`_ from k-okada/hydro
  upgrade to hydro
* upgrade to hydro
* added rosdep eigen
* removed eigen rosdep
* updated version
* fixed rosdep
* Switched linear kernel to RBF
* Moved to github
* Contributors: Dave Coleman, Hunter Laux, Kei Okada, Scott Niekum, sniekum
