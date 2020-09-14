cmake_minimum_required(VERSION 2.8.3)
project(nmea_to_geopose)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geodesy
  geographic_msgs
  nmea_msgs
  roscpp
  quaternion_operation
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES nmea_to_geopose
  CATKIN_DEPENDS geodesy geographic_msgs nmea_msgs roscpp quaternion_operation
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(nmea_to_geopose_node src/nmea_to_geopose_node.cpp src/nmea_to_geopose.cpp)
target_link_libraries(nmea_to_geopose_node ${catkin_LIBRARIES} glog)

# install executables
install(TARGETS nmea_to_geopose_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install launch files
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)