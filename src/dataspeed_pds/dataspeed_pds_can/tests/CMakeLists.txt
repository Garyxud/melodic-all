### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(roslaunch REQUIRED) # Needed for ROS_DISTRO environment variable
find_package(rostest REQUIRED)

# Check all the launch/*.launch files
roslaunch_add_file_check(../launch)

# Generate CAN messages for each module
add_executable(${PROJECT_NAME}_hz hz.cpp)
target_link_libraries(${PROJECT_NAME}_hz ${catkin_LIBRARIES})

# Test frequency with multiple modules
add_rostest(hz.test ARGS unit_id:=0)
add_rostest(hz.test ARGS unit_id:=1)
add_rostest(hz.test ARGS unit_id:=2)
add_rostest(hz.test ARGS unit_id:=3)

