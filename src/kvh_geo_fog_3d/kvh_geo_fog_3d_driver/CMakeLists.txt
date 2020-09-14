cmake_minimum_required(VERSION 2.8.3)
project(kvh_geo_fog_3d_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  kvh_geo_fog_3d_msgs
  message_generation
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf2_ros
)

set(KVH_DRIVER_FILES
  src/kvh_driver/driver_main.cpp
  src/kvh_driver/decode_packets.cpp
  src/kvh_driver/packet_storage.cpp
  src/kvh_driver/kvh_global_vars.cpp
  src/kvh_driver/device_configuration.cpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

##
# For this package, message live in the sibling "_msgs" package, so don't add any
# here.
##

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/${PROJECT_NAME}.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs kvh_geo_fog_3d_msgs message_runtime nav_msgs roscpp sensor_msgs std_msgs tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include include/geo_fog_sdk include/rs232
  ${catkin_INCLUDE_DIRS}
)

## Notes on the below:
## I would like to build the sdk as a seperate library, instead of building it 
## in with the ros library. When built seperately, I run into the issue of tcgetattribute
## causing a segmentation fault. There must be some cmake way around this, but I do not 
## know or want to spend time on it right now.
#add_library(geo_fog_sdk
#  geo_fog_sdk/an_packet_protocol.c
#  geo_fog_sdk/spatial_packets.c
#  geo_fog_sdk/rs232/rs232.c 
#)

add_library(${PROJECT_NAME}
  geo_fog_sdk/an_packet_protocol.c
  geo_fog_sdk/rs232/rs232.c
  geo_fog_sdk/spatial_packets.c
  src/kvh_diagnostics_container.cpp
  ${KVH_DRIVER_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/kvh_geo_fog_3d_node.cpp
  ${KVH_DRIVER_FILES}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


### Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# For determine_baud_node.cpp
add_executable(determine_baud_node src/utility/determine_baud_node.cpp)
add_dependencies(determine_baud_node
  ${PROJECT_NAME}
)
target_link_libraries(determine_baud_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#  ${PROJECT_NAME}_gencfg
#)

#configure_file(src/config.txt ${CMAKE_CURRENT_BINARY_DIR}/config.txt COPYONLY)
#file(COPY settings DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/flir_adk_driver_node.dir/src)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node determine_baud_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Install our header files into the root include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
  )
## Install KVH SDK headers as well
install(DIRECTORY include/geo_fog_sdk
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
## Install RS232 header as well
install(DIRECTORY include/rs232
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
## Make sure we get everything in launch installed
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install our scripts
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  PATTERN ".gitignore" EXCLUDE
)

#############
## Testing ##
#############

## Add roslint target
# Note that roslint requires explicit lists of source/header
# files, which is not ideal. There's an issue open from 2014
# about this (https://github.com/ros/roslint/issues/27)
#
# Ignore ROS's suggestions for:
# * Comment spacing
# * Parentesis whitespace
set(ROSLINT_CPP_OPTS "--filter=-whitespace/comments,-whitespace/parens")
roslint_cpp(
  src/kvh_diagnostics_container.cpp
  src/kvh_geo_fog_3d_node.cpp
  src/kvh_driver/decode_packets.cpp
  src/kvh_driver/device_configuration.cpp
  src/kvh_driver/driver_main.cpp
  src/kvh_driver/kvh_global_vars.cpp
  src/kvh_driver/packet_storage.cpp
  src/utility/determine_baud_node.cpp
  include/kvh_diagnostics_container.hpp
  include/kvh_geo_fog_3d_device_configuration.hpp
  include/kvh_geo_fog_3d_driver.hpp
  include/kvh_geo_fog_3d_global_vars.hpp
  include/kvh_geo_fog_3d_packet_storage.hpp
)

## Add gtest based cpp test target and link libraries
include_directories(
  test/include
  )

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test 
    test/init_vars.cpp
    test/device_configuration_test.cpp
    test/packet_storage_test.cpp
    test/kvh_driver_test.cpp
    )
endif()
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
