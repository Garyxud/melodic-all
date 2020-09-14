^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aws_ros1_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2019-08-01)
------------------
* increment patch version (`#26 <https://github.com/aws-robotics/utils-ros1/issues/26>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Add gtest and gmock as test dependencies (`#17 <https://github.com/aws-robotics/utils-ros1/issues/17>`_)
  * Add gtest and gmock as test dependencies
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * more CMakeLists.txt cleanup
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Remove legacy portions of the ParameterReader API (`#9 <https://github.com/aws-robotics/utils-ros1/issues/9>`_)
* Update ParameterReader API to support ROS1/ROS2 (`#8 <https://github.com/aws-robotics/utils-ros1/issues/8>`_)
  * Revert "Revert ParameterReader change (`#5 <https://github.com/aws-robotics/utils-ros1/issues/5>`_)"
  * refactor based on new ParameterPath object design
* Revert ParameterReader change (`#5 <https://github.com/aws-robotics/utils-ros1/issues/5>`_)
  * Revert "Parameter Namespacing: Refactoring using the ParameterPath object. (`#3 <https://github.com/aws-robotics/utils-ros1/issues/3>`_)"
  This reverts commit 295f157b32a321e230ef1c7f616ad5abd1bece5e.
  https://github.com/aws-robotics/utils-common/issues/15
* Parameter Namespacing: Refactoring using the ParameterPath object. (`#3 <https://github.com/aws-robotics/utils-ros1/issues/3>`_)
  * Refactoring using the ParameterPath object.
  * Minor test fixes in parameter_reader_test.cpp
  * Adding failure test case for Ros1NodeParameterReader.
  * Bumping major version in package.xml
  * address comments in PR
  * update .travis.yml to build dependencies from latest source
* Contributors: AAlon, M. M, Ross Desmond
