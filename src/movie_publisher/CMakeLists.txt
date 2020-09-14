cmake_minimum_required(VERSION 2.8.3)
project(movie_publisher)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS rosbash_params sensor_msgs)

install(PROGRAMS
  nodes/movie_publisher_node
  scripts/add_movie_to_bag
  scripts/fix_bag_timestamps
  scripts/merge.py
  scripts/movie_to_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/movie_publisher.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)