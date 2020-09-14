##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(scheduler_msgs)

##############################################################################
# Dependencies
##############################################################################

set(${PROJECT_NAME}_MSG_DEPENDENCIES
    rocon_std_msgs
    std_msgs
    uuid_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation ${${PROJECT_NAME}_MSG_DEPENDENCIES})

##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  CurrentStatus.msg
  KnownResources.msg
  Request.msg
  Resource.msg
  SchedulerRequests.msg)

generate_messages(DEPENDENCIES ${${PROJECT_NAME}_MSG_DEPENDENCIES})

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS message_runtime ${${PROJECT_NAME}_MSG_DEPENDENCIES})
