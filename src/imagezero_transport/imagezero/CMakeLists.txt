cmake_minimum_required(VERSION 2.8.3)

project(imagezero)

find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

set(iz_SRCS
    src/file.cpp
    src/libiz.cpp
    src/portableimage.cpp
    src/table.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-DHAVE_MMAP)

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)

message(STATUS "Architecture: ${ARCHITECTURE}")

if(${ARCHITECTURE} STREQUAL "x86_64")
  set(ARCH_SPECIFIC_FLAGS "-mmmx -mtune=corei7-avx")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARCH_SPECIFIC_FLAGS} -Winline -W -Wall -fomit-frame-pointer -fno-rtti -fno-exceptions")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-O1,-s,--no-export-dynamic")

add_executable(image_zero
  ${iz_SRCS} 
  src/main.cpp
  )

add_library(${PROJECT_NAME} 
  ${iz_SRCS}
  )

install(TARGETS ${PROJECT_NAME} image_zero
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
