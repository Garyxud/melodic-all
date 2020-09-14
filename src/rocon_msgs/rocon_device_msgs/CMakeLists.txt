##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_device_msgs)

##############################################################################
# Dependencies
##############################################################################

set(${PROJECT_NAME}_MSG_DEPENDENCIES
    std_msgs
    rocon_std_msgs
)

find_package(catkin REQUIRED
  COMPONENTS message_generation ${${PROJECT_NAME}_MSG_DEPENDENCIES})

##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  Hue.msg
  HueState.msg
  HueArray.msg
  Device.msg
  Devices.msg
  )

add_service_files(
  DIRECTORY srv
  FILES
  GetDeviceList.srv
  )

generate_messages(DEPENDENCIES ${${PROJECT_NAME}_MSG_DEPENDENCIES})

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS message_runtime ${${PROJECT_NAME}_MSG_DEPENDENCIES})
