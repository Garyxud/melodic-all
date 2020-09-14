cmake_minimum_required(VERSION 2.8.3)
project(cob_android_script_server)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/script_server_android
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
