cmake_minimum_required(VERSION 3.0.2)
project(dataspeed_pds_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  Channel.msg
  Common.msg
  Inverter.msg
  Mode.msg
  Relay.msg
  Script.msg
  Status.msg
  Temperature.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)

install(DIRECTORY bmr
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bmr"
)
