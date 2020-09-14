# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(network_control_tests)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS network_monitor_udp network_traffic_control hostapd_access_point linksys_access_point ddwrt_access_point access_point_control dynamic_reconfigure rostest)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

#target_link_libraries(${PROJECT_NAME} another_library)
#
#generate_messages(
#    #TODO DEPENDENCIES network_monitor_udp network_traffic_control hostapd_access_point linksys_access_point ddwrt_access_point access_point_control dynamic_reconfigure rostest
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS network_monitor_udp network_traffic_control hostapd_access_point linksys_access_point ddwrt_access_point access_point_control dynamic_reconfigure rostest
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
