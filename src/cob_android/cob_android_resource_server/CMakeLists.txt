cmake_minimum_required(VERSION 2.8.3)
project(cob_android_resource_server)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

catkin_python_setup()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/resource_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)