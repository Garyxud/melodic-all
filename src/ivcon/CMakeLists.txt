cmake_minimum_required(VERSION 2.8.3)
project(ivcon)

find_package(catkin REQUIRED)

catkin_package()

add_executable(${PROJECT_NAME} src/ivcon.c)
target_link_libraries(${PROJECT_NAME} m)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin)
