cmake_minimum_required(VERSION 2.8.3)
project(haros_catkin)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_destinations()

catkin_generate_virtualenv()

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
  EXPORTED_TARGETS ${PROJECT_NAME}_generate_virtualenv
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
