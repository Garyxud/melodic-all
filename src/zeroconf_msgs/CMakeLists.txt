##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(zeroconf_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  DiscoveredService.msg
  Protocols.msg
  PublishedService.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  AddListener.srv
  AddService.srv
  ListDiscoveredServices.srv
  ListPublishedServices.srv
  RemoveListener.srv
  RemoveService.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

