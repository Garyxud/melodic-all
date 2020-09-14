^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aws_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2020-03-27)
------------------
* Update package.xml
  https://github.com/aws-robotics/utils-common/pull/52 adds a new feature for expanding file paths
* Fix linting issues found by clang-tidy 6.0 (`#53 <https://github.com/aws-robotics/utils-common/issues/53>`_)
  * fix linter errors
  * fix more linter errors
  * add linter exceptions
  * address PR comments
* Add wordexp_ros wrapper for wordexp, allowing ROS-aware path expansion
* Add buildtool dependency on ament_cmake_gtest/gmock
* Update DefineTestMacros.cmake
* Add NoRetryStrategy (`#38 <https://github.com/aws-robotics/utils-common/issues/38>`_)
  * ROS-2222: Add Configuration For Retry Strategy
  cr https://code.amazon.com/reviews/CR-10006070
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Don't allow max_retries to override strategy
  cr https://code.amazon.com/reviews/CR-10283519
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * cleanup NoRetryStrategy
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * increment minor version
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Fix ament macro call in DefineTestMacros.cmake (`#35 <https://github.com/aws-robotics/utils-common/issues/35>`_)
  * Fix ament macro call in DefineTestMacros.cmake
  * Disallow Travis build failures for dashing
* Add macro for ros1/2 finding gtest and gmock (`#30 <https://github.com/aws-robotics/utils-common/issues/30>`_)
  * Add macro for ros1/2 finding gtest and gmock
  The macro `find_common_test_packages` will use ament or catkin to link to gtest and gmock libraries.
  **Note:** You must add dependencies on gtest and gmock in the package.xml still
  * remove linkage against redundant library
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml test matrix
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * make catkin a buildtool_depend
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Contributors: AAlon, Avishay Alon, M. M, Miaofei Mei, Ross Desmond, Ryan Newell

2.0.0 (2019-02-25)
------------------
* Add ROS2 dependencies to package.xml (`#21 <https://github.com/aws-robotics/utils-common/issues/21>`_)
  * Update package.xml to depend on ament_cmake_gtest and ament_cmake_gmock if building for ROS2.
* Disallow use of non-ParameterPath objects in ParameterReaderInterface (`#19 <https://github.com/aws-robotics/utils-common/issues/19>`_)
* Remove legacy portions of the ParameterReader API (`#18 <https://github.com/aws-robotics/utils-common/issues/18>`_)
  * remove lunar travis builds
  * remove legacy portions of the ParameterReader API
* Update ParameterReader API to support ROS1/ROS2 (`#17 <https://github.com/aws-robotics/utils-common/issues/17>`_)
  * cleanup CMakeFiles
  * refactor using the new ParameterReader API
  * clean up design of ParameterPath object
* Fix tests not running & optimize build time (`#13 <https://github.com/aws-robotics/utils-common/issues/13>`_)
* Merge pull request `#9 <https://github.com/aws-robotics/utils-common/issues/9>`_ from xabxx/master
  fixed throttling manager unit test bug
* Update local variable name to class member name
  throttled_function_call_count -> throttled_function_call_count\_
* Improve test coverage
* fixed throttling manager unit test bug
* Contributors: AAlon, Abby Xu, M. M, Ross Desmond, hortala
