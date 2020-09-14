cmake_minimum_required(VERSION 2.8.3)
project(ouster_driver)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  roscpp
)

add_message_files(DIRECTORY msg FILES PacketMsg.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ouster_driver
  CATKIN_DEPENDS pcl_ros pcl_conversions roscpp message_runtime std_msgs sensor_msgs
)

add_executable(${PROJECT_NAME} src/os1_node.cpp src/os1_ros.cpp src/os1.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

install(TARGETS
            ${PROJECT_NAME}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        PATTERN ".svn" EXCLUDE
        )

