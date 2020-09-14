cmake_minimum_required(VERSION 3.0.2)
project(dataspeed_pds_scripts)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
