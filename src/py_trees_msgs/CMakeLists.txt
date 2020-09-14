##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(py_trees_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    dynamic_reconfigure
    message_generation
    std_msgs
    uuid_msgs
)

##############################################################################
# Messages
##############################################################################

generate_dynamic_reconfigure_options(
    dynamic_reconfigure/Battery.cfg
    dynamic_reconfigure/MockActionServer.cfg
    dynamic_reconfigure/MockSafetySensors.cfg
)

add_action_files(
  DIRECTORY
      action
  FILES
      Dock.action
      Rotate.action
)

add_message_files(
  DIRECTORY
      msg
  FILES
      Behaviour.msg
      BehaviourTree.msg
)

add_service_files(
  DIRECTORY
      srv
  FILES
      GetBlackboardVariables.srv
      OpenBlackboardWatcher.srv
      CloseBlackboardWatcher.srv
)

generate_messages(DEPENDENCIES
    actionlib_msgs
    std_msgs
    uuid_msgs
)

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS
    message_runtime
    std_msgs
    uuid_msgs
)
