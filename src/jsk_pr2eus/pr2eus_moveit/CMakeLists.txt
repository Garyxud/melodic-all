# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2eus_moveit)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pr2eus moveit_msgs control_msgs roseus)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES std_msgs control_msgs moveit_msgs
# )

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS
    CATKIN_DEPENDS pr2eus moveit_msgs control_msgs roseus
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY euslisp tutorials
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(pr2_gazebo QUIET)
  if(pr2_gazebo_FOUND)
    add_rostest(test/test-pr2eus-moveit.test)
  else()
    message(WARNING "pr2_gazebo is not found. Skipping pr2eus_moveit tests")
  endif()
endif()
