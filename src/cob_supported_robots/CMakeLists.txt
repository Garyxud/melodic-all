cmake_minimum_required(VERSION 2.8.3)
project(cob_supported_robots)

find_package(catkin REQUIRED)

set(robotlist
    cob4-3
    cob4-4
    cob4-5
    cob4-6
    cob4-7
    cob4-8
    cob4-9
    cob4-10
    cob4-11
    cob4-13
    cob4-16
    cob4-18
    cob4-20
    cob4-23
    cob4-24
    cob4-25
    raw3-1
    raw3-3
    raw3-5
)

catkin_package(
    CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)
