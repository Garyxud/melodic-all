# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_gripper_sensor_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib message_generation std_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(DIRECTORY msg
  FILES
  PR2GripperEventDetectorCommand.msg
  PR2GripperEventDetectorData.msg
  PR2GripperFindContactCommand.msg
  PR2GripperFindContactData.msg
  PR2GripperPressureData.msg
  PR2GripperReleaseCommand.msg
  PR2GripperReleaseData.msg
  PR2GripperSensorRawData.msg
  PR2GripperSensorRTState.msg
  PR2GripperSlipServoCommand.msg
  PR2GripperSlipServoData.msg
  PR2GripperGrabData.msg
  PR2GripperGrabCommand.msg
  PR2GripperForceServoCommand.msg
  PR2GripperForceServoData.msg
)


add_action_files(DIRECTORY action
  FILES
  PR2GripperEventDetector.action
  PR2GripperFindContact.action
  PR2GripperForceServo.action
  PR2GripperGrab.action
  PR2GripperRelease.action
  PR2GripperSlipServo.action
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_INCLUDE_DIRS})
#uncomment if you have defined services
#add_service_files(

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs#TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS include roscpp actionlib_msgs actionlib std_msgs
    CATKIN_DEPENDS actionlib_msgs message_runtime# TODO
    INCLUDE_DIRS ${catkin_INCLUDE_DIRS}# TODO include
    LIBRARIES # TODO
)

