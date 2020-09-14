# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(network_monitor_udp)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp rospy diagnostic_msgs actionlib_msgs actionlib message_generation)

# include python modules
catkin_python_setup()

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})




#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
add_message_files(
    FILES 
    UdpSink.msg
    UdpMonitor.msg
)

add_action_files(
    FILES 
    Linktest.action
)

generate_messages(
    DEPENDENCIES diagnostic_msgs actionlib_msgs actionlib
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy diagnostic_msgs actionlib_msgs actionlib message_runtime 
    #CATKIN_DEPENDS # TODO
    #INCLUDE_DIRS # TODO include
    #LIBRARIES # TODO
)

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(udpmonserv src/utilities/udpmonserv.c src/utilities/helpers.c)
add_executable(udpmonsink src/utilities/udpmonsink.cpp src/utilities/helpers.c)
target_link_libraries(udpmonserv ${catkin_LIBRARIES})
target_link_libraries(udpmonsink ${catkin_LIBRARIES})
add_dependencies(udpmonsink ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(udpmonserv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS udpmonserv udpmonsink
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS nodes/network_watchdog.py nodes/sample_bwtest.py nodes/udpmonclinode.py nodes/udpmonsourcenode.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES launch_nodes.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
