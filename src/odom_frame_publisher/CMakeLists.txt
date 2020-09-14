cmake_minimum_required(VERSION 2.8.3)
project(odom_frame_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  quaternion_operation
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES odom_frame_publisher
  CATKIN_DEPENDS geometry_msgs roscpp tf2_ros tf2 tf2_msgs tf2_geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(odom_frame_publisher_node src/odom_frame_publisher_node.cpp src/odom_frame_publisher.cpp)
target_link_libraries(odom_frame_publisher_node ${catkin_LIBRARIES})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir launch)
install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Install library
install(TARGETS odom_frame_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)