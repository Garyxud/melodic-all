### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(roslaunch REQUIRED)

# Check all the launch/*.launch files
roslaunch_add_file_check(../launch)

# Unit tests
catkin_add_gtest(${PROJECT_NAME}_test_module_version test_module_version.cpp)
target_link_libraries(${PROJECT_NAME}_test_module_version ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_pedal_lut test_pedal_lut.cpp)
target_link_libraries(${PROJECT_NAME}_test_pedal_lut ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_platform_map test_platform_map.cpp)
target_link_libraries(${PROJECT_NAME}_test_platform_map ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_platform_version test_platform_version.cpp)
target_link_libraries(${PROJECT_NAME}_test_platform_version ${catkin_LIBRARIES})

