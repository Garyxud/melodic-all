cmake_minimum_required(VERSION 2.8.3)
project(parameter_assertions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-W -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

set(PARAMETER_ASSERTIONS_LIB parameter_assertions)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PARAMETER_ASSERTIONS_LIB}
    CATKIN_DEPENDS roscpp
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PARAMETER_ASSERTIONS_LIB} src/assertions.cpp)
target_link_libraries(${PARAMETER_ASSERTIONS_LIB} ${catkin_LIBRARIES})

install(
    TARGETS ${PARAMETER_ASSERTIONS_LIB}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
endif ()
