cmake_minimum_required(VERSION 2.8.3)
project(quaternion_operation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quaternion_operation
  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(quaternion_operation src/quaternion_operation.cpp)
target_link_libraries(quaternion_operation ${catkin_LIBRARIES})

# install executables/libs
install(TARGETS quaternion_operation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-quaternion_operation
  test/test_quaternion_operation.test
  test/src/test_quaternion_operation.cpp
  ${NODE_STATUS_PUBLISHER_SRC})
  target_link_libraries(test-quaternion_operation
  quaternion_operation
  ${catkin_LIBRARIES})
endif ()