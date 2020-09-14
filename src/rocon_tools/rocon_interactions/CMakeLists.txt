##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_interactions)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roslint)
catkin_package()

catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/rocon_interactions
        scripts/interactions_manager.py
        scripts/load_interactions
        scripts/fake_remocon
        scripts/tutorial_markers.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY interactions DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY js DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
