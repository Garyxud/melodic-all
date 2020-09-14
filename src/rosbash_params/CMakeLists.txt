cmake_minimum_required(VERSION 2.8.3)
project(rosbash_params)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS rosbash)

install(FILES ros_params_parser.bash DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_add_env_hooks(30.rosbash_params SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)