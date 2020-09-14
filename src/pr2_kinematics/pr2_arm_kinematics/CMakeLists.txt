# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_arm_kinematics)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED COMPONENTS roscpp angles tf_conversions urdf geometry_msgs kdl_parser moveit_msgs moveit_core pluginlib cmake_modules)
find_package(Eigen REQUIRED)


include_directories(include ${catkin_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
 
catkin_package(
    DEPENDS roscpp angles tf_conversions urdf geometry_msgs kdl_parser moveit_msgs pluginlib eigen moveit_core
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include# TODO include
    LIBRARIES pr2_arm_kinematics_lib# TODO
)

add_library(pr2_arm_kinematics_lib
  src/pr2_arm_ik.cpp
  src/pr2_arm_ik_solver.cpp
  src/pr2_arm_kinematics_utils.cpp
  src/pr2_arm_kinematics.cpp
  src/pr2_arm_kinematics_plugin.cpp
)

add_executable(pr2_arm_kinematics_node src/main.cpp)
target_link_libraries(pr2_arm_kinematics_node pr2_arm_kinematics_lib ${catkin_LIBRARIES})
target_link_libraries(pr2_arm_kinematics_lib ${catkin_LIBRARIES})

install(FILES test_pr2_arm_ik.xml test_pr2_arm_ik_node.xml kinematics_plugins.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS pr2_arm_kinematics_lib 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS pr2_arm_kinematics_node 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
