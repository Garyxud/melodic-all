cmake_minimum_required(VERSION 2.8.3)
project(planner_cspace_msgs)

set(MESSAGE_DEPENDS
  actionlib_msgs
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDS})

add_action_files(
  DIRECTORY
    action
  FILES
    MoveWithTolerance.action
)

add_message_files(
  FILES
    PlannerStatus.msg
)
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDS})
catkin_package(CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDS})
