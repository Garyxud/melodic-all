cmake_minimum_required(VERSION 2.8.3)
project(rosbag_pandas)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/bag_csv
  scripts/bag_plot
  scripts/bag_print
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()