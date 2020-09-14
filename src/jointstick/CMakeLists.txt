cmake_minimum_required(VERSION 2.8.3)
project(jointstick)

find_package(catkin REQUIRED COMPONENTS
    controller_manager_msgs
    trajectory_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
)

catkin_package()

include_directories()
