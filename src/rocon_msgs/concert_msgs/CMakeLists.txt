##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    gateway_msgs
    message_generation 
    rocon_app_manager_msgs
    rocon_std_msgs
    std_msgs
    uuid_msgs
)

##############################################################################
# Messgaes
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  ConcertClient.msg
  ConcertClients.msg
  ConcertClientState.msg
  ConductorGraph.msg
  ErrorCodes.msg
  LinkConnection.msg
  LinkEdge.msg
  LinkGraph.msg
  LinkNode.msg
  ServiceProfile.msg
  Services.msg
  SoftwareProfile.msg
  SoftwareProfiles.msg
  SoftwareInstance.msg
  SoftwareInstances.msg
  Strings.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  EnableService.srv
  UpdateServiceConfig.srv
  AllocateSoftware.srv
)

generate_messages(DEPENDENCIES
    gateway_msgs
    rocon_app_manager_msgs
    rocon_std_msgs
    std_msgs 
    uuid_msgs
)

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS 
                   gateway_msgs
                   message_runtime
                   rocon_app_manager_msgs
                   rocon_std_msgs
                   std_msgs
                   uuid_msgs
               )
