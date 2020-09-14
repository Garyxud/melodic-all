cmake_minimum_required(VERSION 2.8.3)
project(nmea_comms)

find_package(catkin REQUIRED COMPONENTS roscpp roslaunch roslint nmea_msgs rostest)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}_lib src/rx.cpp src/tx.cpp src/checksum.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(nmea_serial_node src/serial_node.cpp)
set_target_properties(nmea_serial_node PROPERTIES OUTPUT_NAME serial_node PREFIX "")
add_dependencies(nmea_serial_node nmea_msgs_gencpp)
target_link_libraries(nmea_serial_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

add_executable(nmea_socket_node src/socket_node.cpp)
set_target_properties(nmea_socket_node PROPERTIES OUTPUT_NAME socket_node PREFIX "")
add_dependencies(nmea_socket_node nmea_msgs_gencpp)
target_link_libraries(nmea_socket_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#############
## Install ##
#############

install(TARGETS nmea_serial_node nmea_socket_node ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(nmea_socket_node_test test/socket_node.test test/test_socket_node.cpp)
  target_link_libraries(nmea_socket_node_test ${catkin_LIBRARIES})
  add_dependencies(nmea_socket_node_test nmea_socket_node nmea_msgs_gencpp)

  roslaunch_add_file_check(launch/tee.launch
    DEPENDENCIES nmea_socket_node nmea_serial_node)

  roslint_cpp()
endif()
