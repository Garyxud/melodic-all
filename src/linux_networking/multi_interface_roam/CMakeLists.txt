# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(multi_interface_roam)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rospy network_monitor_udp std_msgs diagnostic_msgs pr2_msgs dynamic_reconfigure ieee80211_channels)

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


#rosbuild_gensrv()

catkin_add_nosetests(src/multi_interface_roam/event.py)
catkin_add_nosetests(src/multi_interface_roam/state_publisher.py)
catkin_add_nosetests(src/multi_interface_roam/async_helpers.py)
catkin_add_nosetests(src/multi_interface_roam/command_with_output.py)
catkin_add_nosetests(src/multi_interface_roam/system.py)
catkin_add_nosetests(src/multi_interface_roam/mac_addr.py)


add_message_files(
    FILES 
    InterfaceStatus.msg
    MultiInterfaceStatus.msg
)



#generate_messages(
#    #TODO DEPENDENCIES rospy network_monitor_udp std_msgs diagnostic_msgs pr2_msgs wpa_supplicant_node asmach dynamic_reconfigure ieee80211_channels python-libpcap python-twisted-core udhcpc python-scapy beep
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy network_monitor_udp std_msgs diagnostic_msgs pr2_msgs wpa_supplicant_node dynamic_reconfigure ieee80211_channels
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
