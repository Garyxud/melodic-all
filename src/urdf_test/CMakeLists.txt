cmake_minimum_required(VERSION 2.8.3)
project(urdf_test)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urdf_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS gtest
)

#############
## Install ##
#############
install(PROGRAMS
    scripts/test_urdf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
