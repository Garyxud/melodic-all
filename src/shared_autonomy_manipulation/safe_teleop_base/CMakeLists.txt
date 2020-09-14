cmake_minimum_required(VERSION 2.8.3)
project(safe_teleop_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp base_local_planner costmap_2d)

catkin_package(
  CATKIN_DEPENDS roscpp base_local_planner costmap_2d
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(safe_teleop_base src/safe_trajectory_planner_ros.cpp src/safe_trajectory_planner.cpp)
target_link_libraries(safe_teleop_base ${catkin_LIBRARIES})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(TARGETS safe_teleop_base ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
