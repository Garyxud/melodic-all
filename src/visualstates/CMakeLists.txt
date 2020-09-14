cmake_minimum_required(VERSION 2.8)
project(visualstates)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_python_setup()
catkin_package(INCLUDE_DIRS include)

SET(SOURCE_CPP_RUNTIME
	src/visualstates/codegen/cpp/state.cpp
	src/visualstates/codegen/cpp/transition.cpp
	src/visualstates/codegen/cpp/temporaltransition.cpp
    src/visualstates/codegen/cpp/runtimegui.cpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime pthread)
INSTALL(TARGETS visualStatesRunTime LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS scripts/main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/visualstates/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install cpp sources for ros node generation
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/visualstates/codegen/cpp/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/codegen)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
