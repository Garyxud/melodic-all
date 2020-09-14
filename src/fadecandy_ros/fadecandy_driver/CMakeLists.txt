cmake_minimum_required(VERSION 2.8.3)
project(fadecandy_driver)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
    scripts/fadecandy_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
