cmake_minimum_required(VERSION 2.8.3)
project(pyros_utils)

# For ease of use, this CMakeLists allows to use this code from source, as if it were a normal catkin package.
# However no ROS/catkin debian package is provided for it.
# It is just a pip package that your packages should depend on (using rosdep pip dependency mechanism).
# We should follow closely https://github.com/gerkey/ros1_external_use

# Minimal Python module setup - catkin is assumed preinstalled along with ROS
# This assume catkin is installed along with a minimal ROS system.
find_package(catkin REQUIRED COMPONENTS
    catkin_pip
    rospy
    rosgraph
    roslaunch
    rostest
)

# defining current package as a package that should be managed by pip (not catkin - even though we make it usable with workspaces)
catkin_pip_package(pyros_utils)

## Unit tests
if (CATKIN_ENABLE_TESTING)

    ############
    # Nose Tests
    ############

    catkin_add_nosetests(pyros_utils/tests)
    # this test needs to be executable for rostest, so we need to add it specifically for nose
    catkin_add_nosetests(pyros_utils/tests/test_rostest_nose.py)

    ###########
    # ROS Tests
    ###########

    find_package(rostest REQUIRED)
    add_rostest(pyros_utils/tests/test_rostest_nose.test)

endif()
