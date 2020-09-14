cmake_minimum_required(VERSION 2.8.3)
project(criutils)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

# Install python scripts
catkin_install_python(
  PROGRAMS
    scripts/tf_transform_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
