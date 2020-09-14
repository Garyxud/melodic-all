^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lex_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2019-08-29)
------------------
* Merge pull request `#4 <https://github.com/aws-robotics/lex-common/issues/4>`_ from aws-robotics/ros2_migration
  Cleanup CMakeLists.txt and README
* recover test code coverage
  Signed-off-by: Miaofei <miaofei@amazon.com>
* cleanup cmake file to improve compatbility with ros1 and ros2
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Change ament to ament_cmake
* Change ament to ament_cmake
* Merge pull request `#1 <https://github.com/aws-robotics/lex-common/issues/1>`_ from aws-robotics/lex-dev
  Lex Initial Development
* Revise code to resolve pull request comments
  **Summary**
  * Add ament dependencies in package.xml
  * Remove incompatible test from compiling with catkin
  * Revise README.md with up to date instructions
* Sync ParameterPath with utils-ros2
* Ament uncrustify tests
* Increase code coverage
  **Code Coverage Summary**
  lex_common.h                                   | 100%    16| 100%  11|
  lex_configuration.h                            | 100%     1| 100%   2|
  lex_common.cpp                                 |85.5%    76|85.7%   7|
  lex_param_helper.cpp                           | 100%    12| 100%   1|
* Add tests for lex_common to Aws Lex
* Add lex common interfaces for ros1 and ros2
  **Summary**
  Creates readme for how to build lex common
  Interface can be used as a library for any C++ usage of Lex
  Ament_uncrustify reformat code
  ament_cpplint formatting changes
  **Tests**
  Unit Tests
* Add lex common interfaces for ros1 and ros2
  **Summary**
  Creates readme for how to build lex common
  Interface can be used as a library for any C++ usage of Lex
  Ament_uncrustify reformat code
  ament_cpplint formatting changes
  **Tests**
  Unit Tests
* Contributors: AAlon, M. M, Miaofei, Ross Desmond
