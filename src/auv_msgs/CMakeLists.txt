cmake_minimum_required(VERSION 3.0.2)
project(auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
    BodyForceRequest.msg
    BodyVelocityRequest.msg
    NavigationStatus.msg
    NED.msg
    NEDArray.msg
    VehiclePose.msg
    VehiclePoseArray.msg
    WorldWaypointRequest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geographic_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    message_runtime
)
