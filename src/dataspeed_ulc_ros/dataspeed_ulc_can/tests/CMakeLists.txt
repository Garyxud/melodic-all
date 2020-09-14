### Tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

find_package(roslib REQUIRED)
find_package(rostest REQUIRED)

# Test node
add_rostest_gtest(${PROJECT_NAME}_test_node test_ulc_node.test test_ulc_node.cpp)
add_dependencies(${PROJECT_NAME}_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_node ${catkin_LIBRARIES})

# Test nodelet
add_rostest_gtest(${PROJECT_NAME}_test_nodelet test_ulc_nodelet.test test_ulc_node.cpp)
add_dependencies(${PROJECT_NAME}_test_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_nodelet ${catkin_LIBRARIES})

# Test config frequency parameter (for ROS distributions newer than Indigo)
if($ENV{ROS_DISTRO} STRGREATER indigo)
  add_rostest(param_cfg_freq.test ARGS config_freq:=2  param_value:=5)
  add_rostest(param_cfg_freq.test ARGS config_freq:=20 param_value:=20)
  add_rostest(param_cfg_freq.test ARGS config_freq:=99 param_value:=50)
endif()
