cmake_minimum_required(VERSION 2.8.3)
project(costmap_cspace_msgs)

set(MESSAGE_DEPENDS
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDS})

add_message_files(
  FILES
    CSpace3D.msg
    CSpace3DUpdate.msg
    MapMetaData3D.msg
)
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDS})
catkin_package(CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDS})
