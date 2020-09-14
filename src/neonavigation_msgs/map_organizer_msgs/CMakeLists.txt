cmake_minimum_required(VERSION 2.8.3)
project(map_organizer_msgs)

set(MESSAGE_DEPENDS
  nav_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDS})

add_message_files(
  FILES
    OccupancyGridArray.msg
)
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDS})
catkin_package(CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDS})
