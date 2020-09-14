##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_service_pair_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    uuid_msgs
)

##############################################################################
# Package
##############################################################################

# This is a bit awkward. Would like to generate pair msgs inside this package
# for rocon_service_pair tests and also need to bring in uuid_msgs as a msg
# dependency (doing so only via package.xml is not sufficient for deeper 
# message generation calls). So a fake generation here of a request/response
# message pair fills both needs.

add_message_files(
  DIRECTORY msg
  FILES
  TestiesPair.msg
  TestiesPairRequest.msg
  TestiesPairResponse.msg
  TestiesRequest.msg
  TestiesResponse.msg
)

generate_messages(DEPENDENCIES uuid_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime uuid_msgs
    CFG_EXTRAS rocon_service_pair_msgs-extras.cmake
    )

##############################################################################
# Installs
##############################################################################

# install the .pair -> .msg generator
install(PROGRAMS scripts/genpair.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
