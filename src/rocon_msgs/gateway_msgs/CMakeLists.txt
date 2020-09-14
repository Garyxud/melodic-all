##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gateway_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    std_msgs 
)

##############################################################################
# Messgaes
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  ConnectionStatistics.msg
  ConnectionType.msg
  GatewayInfo.msg
  RemoteGateway.msg
  RemoteRule.msg
  RemoteRuleWithStatus.msg
  ErrorCodes.msg
  Rule.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  Advertise.srv
  AdvertiseAll.srv
  ConnectHub.srv
  Remote.srv
  RemoteAll.srv
  RemoteGatewayInfo.srv
  SetWatcherPeriod.srv
)

generate_messages(DEPENDENCIES
    std_msgs 
)

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
