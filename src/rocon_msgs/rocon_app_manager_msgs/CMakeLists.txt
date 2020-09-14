##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_app_manager_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    rocon_std_msgs
    rocon_service_pair_msgs
    )

##############################################################################
# Messgaes
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  Rapp.msg
  RappList.msg
  Constants.msg
  ErrorCodes.msg
  IncompatibleRappList.msg
  PublicInterface.msg
  PublishedInterface.msg
  Status.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetRappList.srv
  Init.srv
  Invite.srv
  StartRapp.srv
  StopRapp.srv
)

generate_messages(DEPENDENCIES rocon_std_msgs)

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS 
    message_runtime
    rocon_std_msgs
)
