cmake_minimum_required(VERSION 2.8.3)
project(static_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  tf
  tf2_ros 
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs rospy tf tf2_ros
)

catkin_install_python(PROGRAMS scripts/static_tf DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
