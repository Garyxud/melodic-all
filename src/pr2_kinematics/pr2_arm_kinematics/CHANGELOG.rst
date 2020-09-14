^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_arm_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2018-08-30)
-------------------
* Merge pull request #12 from PR2/add_travis
  add travis.yml and fix for melodic
* fix for kdl v1.4.0 (melodic), c.f. https://github.com/ros-planning/moveit/pull/906
* Contributors: Kei Okada

1.0.9 (2018-02-14)
------------------
* Merge pull request `#10 <https://github.com/pr2/pr2_kinematics/issues/10>`_ from k-okada/use_pose
  PoseMsgToKDL is deperecated use poseMsgToKDL
* Merge pull request `#9 <https://github.com/pr2/pr2_kinematics/issues/9>`_ from k-okada/remove_get_solver_info2
  remove GetKinematicsSolverInfo, whcih is deprecated in kinetic
* Merge pull request `#8 <https://github.com/pr2/pr2_kinematics/issues/8>`_ from k-okada/add_c11
  add c++11 option for error: ‘shared_ptr’ in namespace ‘std’ does not …
* Merge pull request `#7 <https://github.com/pr2/pr2_kinematics/issues/7>`_ from k-okada/orp
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* PoseMsgToKDL is deperecated use poseMsgToKDL
* remove GetKinematicsSolverInfo, whcih is deprecated in kinetic https://github.com/ros-planning/moveit_msgs/issues/3
* add c++11 option for error: ‘shared_ptr’ in namespace ‘std’ does not name a template type typedef std::shared_ptr<Type> Name##Ptr; error
* Contributors: Kei Okada

1.0.7 (2015-02-10)
------------------
* Fix pr2_arm_kinematics for indigo by adding cmake_modules
* Add dependency to cmake_modules to solve Eigen depdency on indigo
* Contributors: Ryohei Ueda

1.0.6 (2014-11-21)
------------------
* Removed mainpage.dox
* Removed mainpage.dox
* Contributors: TheDash, dash

1.0.5 (2014-11-18)
------------------
* Added install target for pr2_kinematics_node to archive and lib dests
* Contributors: TheDash

1.0.4 (2014-11-11)
------------------

1.0.2 (2014-09-07)
------------------
* Removed dependency on kinematics_base, replaced with moveit functionality
* Contributors: TheDash
