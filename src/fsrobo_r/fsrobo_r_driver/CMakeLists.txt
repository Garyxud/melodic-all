cmake_minimum_required(VERSION 2.8.3)
project(fsrobo_r_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  industrial_robot_client
  simple_message
  fsrobo_r_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fsrobo_r_driver
  CATKIN_DEPENDS industrial_robot_client
#  DEPENDS system_lib
)

add_definitions(-DLINUXSOCKETS=1)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fsrobo_r_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(SRC_FILES
  src/fsrobo_r_joint_trajectory_action.cpp
  src/fsrobo_r_joint_trajectory_streamer.cpp
  src/fsrobo_r_robot_state_interface.cpp
  src/fsrobo_r_robot_service_interface.cpp
  src/io_state_relay_handler.cpp
  src/io_control.cpp
  src/robot_program_executor.cpp
  src/robot_configurator.cpp
  src/wrench_relay_handler.cpp
  )
add_library(fsrobo_r_driver ${SRC_FILES})
target_link_libraries(fsrobo_r_driver
  simple_message
  industrial_robot_client
  fsrobo_r_simple_message
  )

set(MSG_SRC_FILES
  src/simple_message/messages/set_io_message.cpp
  src/simple_message/messages/set_io_reply_message.cpp
  src/simple_message/messages/io_state_message.cpp
  src/simple_message/set_io.cpp
  src/simple_message/set_io_reply.cpp
  src/simple_message/io_state.cpp
  src/simple_message/messages/execute_program_message.cpp
  src/simple_message/messages/execute_program_reply_message.cpp
  src/simple_message/execute_program.cpp
  src/simple_message/execute_program_reply.cpp
  src/simple_message/simple_string.cpp
  src/simple_message/posture.cpp
  src/simple_message/messages/set_posture_message.cpp
  src/simple_message/messages/set_posture_reply_message.cpp
  src/simple_message/messages/get_posture_message.cpp
  src/simple_message/messages/get_posture_reply_message.cpp
  src/simple_message/messages/wrench_message.cpp
  src/simple_message/wrench.cpp
  src/simple_message/messages/sys_stat_message.cpp
  src/simple_message/messages/sys_stat_reply_message.cpp
  src/simple_message/sys_stat.cpp
  src/simple_message/sys_stat_reply.cpp
  src/simple_message/messages/set_tool_offset_message.cpp
  src/simple_message/messages/set_tool_offset_reply_message.cpp
  src/simple_message/set_tool_offset.cpp
  src/simple_message/set_tool_offset_reply.cpp
)

# Simple message library
add_library(fsrobo_r_simple_message ${MSG_SRC_FILES})
target_link_libraries(fsrobo_r_simple_message
  simple_message)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fsrobo_r_driver_node.cpp)
add_executable(${PROJECT_NAME}_joint_trajectory_action
  src/fsrobo_r_joint_trajectory_action_node.cpp
)
target_link_libraries(${PROJECT_NAME}_joint_trajectory_action
  industrial_robot_client
  simple_message
  fsrobo_r_driver
  ${catkin_LIBRARIES})
set_target_properties(
  ${PROJECT_NAME}_joint_trajectory_action
  PROPERTIES OUTPUT_NAME joint_trajectory_action PREFIX "")

add_executable(${PROJECT_NAME}_robot_state
  src/fsrobo_r_robot_state_node.cpp)
target_link_libraries(${PROJECT_NAME}_robot_state
  industrial_robot_client
  simple_message
  fsrobo_r_driver
  ${catkin_LIBRARIES})
set_target_properties(
  ${PROJECT_NAME}_robot_state
  PROPERTIES OUTPUT_NAME robot_state PREFIX "")

add_executable(${PROJECT_NAME}_motion_streaming_interface
  src/fsrobo_r_joint_streamer_node.cpp
)
target_link_libraries(${PROJECT_NAME}_motion_streaming_interface
  industrial_robot_client
  simple_message
  fsrobo_r_driver
  ${catkin_LIBRARIES})
set_target_properties(
  ${PROJECT_NAME}_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface PREFIX "")

add_executable(${PROJECT_NAME}_robot_service_interface
  src/fsrobo_r_robot_service_interface_node.cpp
)
target_link_libraries(${PROJECT_NAME}_robot_service_interface
  industrial_robot_client
  simple_message
  fsrobo_r_driver
  ${catkin_LIBRARIES})
set_target_properties(
  ${PROJECT_NAME}_robot_service_interface
  PROPERTIES OUTPUT_NAME robot_service_interface PREFIX "")

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS
  fsrobo_r_driver
  fsrobo_r_simple_message
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}_joint_trajectory_action
  ${PROJECT_NAME}_motion_streaming_interface
  ${PROJECT_NAME}_robot_service_interface
  ${PROJECT_NAME}_robot_state
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/ik_service.py
  scripts/io_gripper_action.py
  scripts/force_publisher.py
  scripts/force_observer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fsrobo_r_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_main.xml)
endif()


# TODO install