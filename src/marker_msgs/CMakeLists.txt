cmake_minimum_required(VERSION 2.8.3)
project(marker_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Marker.msg
  Fiducial.msg
  FiducialDetection.msg
  MarkerDetection.msg
  MarkerStamped.msg
  MarkerWithCovariance.msg
  MarkerWithCovarianceArray.msg
  MarkerWithCovarianceStamped.msg)
  
generate_messages(DEPENDENCIES  geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
