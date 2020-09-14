^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package h264_encoder_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2019-11-01)
------------------
* Update codec opening test to include opts and params (`#37 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/37>`_)
  * Update codec opening test to include opts and params
  * bump package version
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Backoff to software encoding if codec isn't available (`#35 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/35>`_)
  * Backoff to software encoding if codec isn't available
  * Remove is_omx_available
  * Update documentation for running on Raspberr Pi
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Increment version to 2.0.2
* Link libraries only if test target exists
* increment patch version (`#30 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/30>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Add gtest and gmock as test dependencies (`#14 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/14>`_)
  * Add gtest and gmock as test dependencies
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * use the macro in aws_common to find test dependencies for ROS1 or ROS2
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml test matrix
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update PACKAGE_NAMES
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Merge pull request `#22 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/22>`_ from aws-robotics/license-fix
  Moves license file up to the root directory as this licensing applies to the whole repository.
* Moves license file up to the root directory as this licensing applies to the whole repository.
* Update to use non-legacy ParameterReader API (`#9 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/9>`_)
* Update to use new ParameterReader API (`#8 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/8>`_)
  * fix h264_encoder_test to be compatible with new ParameterReader API
  * increment major version number in package.xml
* Merge pull request `#3 <https://github.com/aws-robotics/kinesisvideo-encoder-common/issues/3>`_ from mm318/bionic_hardware_encoder
  workaround avcodec_find_encoder_by_name() unreliably indicating that h264_omx codec is available
* implementing runtime solution for workaround
* workaround avcodec_find_encoder_by_name() unreliably indicating that h264_omx codec is available
* Contributors: AAlon, James Siri, M. M, Miaofei, Nick Burek, Ross Desmond, ryanewel
