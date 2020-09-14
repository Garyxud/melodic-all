cmake_minimum_required(VERSION 2.8.3)
project(tf2_server)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nodelet
  roscpp
  tf2_msgs
  tf2_ros
)

catkin_python_setup()

add_service_files(DIRECTORY srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} tf2_subtree_listener
  CATKIN_DEPENDS geometry_msgs message_runtime nodelet roscpp tf2_msgs tf2_ros
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/tf2_server.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_nodelet src/tf2_server_nodelet.cpp)
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/tf2_server_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(tf2_subtree_listener src/tf2_subtree_listener.cpp)
add_dependencies(tf2_subtree_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf2_subtree_listener ${catkin_LIBRARIES})

if (${CATKIN_ENABLE_TESTING})
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_tf2_subtree_listener test/tf2_subtree_listener.test test/test_tf2_subtree_listener.cpp)
  target_link_libraries(test_tf2_subtree_listener tf2_subtree_listener ${catkin_LIBRARIES})

  add_rostest(test/tf2_subtree_listener_py.test)
  add_rostest(test/tf2_server_initial_params.test)
endif()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet tf2_subtree_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES tf2_server_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)