cmake_minimum_required(VERSION 2.8.3)
project(tuw_gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Measure.msg
  JointsMeasureI4ws.msg
  JointCmd.msg
  JointsCmdI4ws.msg
  JointState.msg
)

generate_messages(DEPENDENCIES std_msgs )

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark executables and/or libraries for installation
#install(TARGETS tuw_gazebo_msgs
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
