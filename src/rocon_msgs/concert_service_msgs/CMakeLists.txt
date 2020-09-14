##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_service_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    rocon_service_pair_msgs
    rocon_std_msgs
)

##############################################################################
# Messages
##############################################################################


#add_service_files(
#  DIRECTORY srv
#  FILES
#)

add_service_pair_files(
  DIRECTORY pairs
  FILES
  CaptureResource.pair
  KillTurtle.pair
  SpawnTurtle.pair
)

generate_messages(DEPENDENCIES rocon_std_msgs rocon_service_pair_msgs)

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS
  message_runtime
  rocon_service_pair_msgs
  rocon_std_msgs
)
