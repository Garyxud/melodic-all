cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_VERBOSE_MAKEFILE ON)
#It seams that from Ubuntu 17.10 the linker flag --disable-new-dtags is needed
#for the executable to be able to find absolute linked shared libraries with rpath.
#https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")

#SET PROJECT NAME
#NOTE: replace "foo" by your project name
SET(PROJECT_NAME_ORIG "class_loader" )

SET(PROJECT_NAME_BASE ${PROJECT_NAME_ORIG} )

message("Building ${PROJECT_NAME_BASE}...")
if(DEFINED ${PROJECT_NAME_BASE}_NAME_OVERRIDE)
    SET(PROJECT_NAME_BASE ${${PROJECT_NAME_BASE}_NAME_OVERRIDE})
endif()

if(DISABLE_GIT_HASH)
	message(STATUS "PROJECT ${PROJECT_NAME_BASE}: DISABLE GIT HASH")
	SET(PROJECT_NAME_GIT "${PROJECT_NAME_BASE}")
else()
	message(STATUS "PROJECT ${PROJECT_NAME_BASE}: GIT HASH ENABLED")
	#BEGIN RESOLVE GIT REVISION
	#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
	message(STATUS "Resolving GIT Version...")
	set(_build_version "unknown")
	find_package(Git)
	if(GIT_FOUND)
	  execute_process(
	    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
	    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
	    OUTPUT_VARIABLE _build_version
	    ERROR_QUIET
	    OUTPUT_STRIP_TRAILING_WHITESPACE
	  )
	  message( STATUS "GIT hash: ${_build_version}")
	else()
	  message(STATUS "GIT not found")
	endif()
	string(TIMESTAMP _time_stamp)
	#END RESOLVE GIT REVISION

	SET(PROJECT_NAME_GIT "${PROJECT_NAME_BASE}_${_build_version}")
endif()

#DECLARE PROJECT
string(TOUPPER ${PROJECT_NAME_ORIG} TMP0)
SET(${TMP0}_GIT "${PROJECT_NAME_GIT}" PARENT_SCOPE)
if (TARGET ${PROJECT_NAME_GIT})
    message(WARNING "${PROJECT_NAME_GIT} has already been built!")
    return()
endif()

project (${PROJECT_NAME_GIT} CXX)

message(STATUS "PROJECT NAME: ${PROJECT_NAME}")

find_package(Boost REQUIRED COMPONENTS thread system)
find_package(console_bridge REQUIRED)

SET(BUILDTYPE ${CMAKE_BUILD_TYPE})
SET(CMAKE_BUILD_TYPE "Release")
set(Poco_DIR cmake)
find_package(Poco REQUIRED COMPONENTS Foundation)
SET(CMAKE_BUILD_TYPE ${BUILDTYPE})
set(CATKIN_GLOBAL_BIN_DESTINATION bin)
set(CATKIN_GLOBAL_LIB_DESTINATION lib)
set(CATKIN_GLOBAL_LIBEXEC_DESTINATION lib)
set(CATKIN_GLOBAL_INCLUDE_DESTINATION include)
set(CATKIN_PACKAGE_LIB_DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})

include_directories(include ${console_bridge_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Poco_INCLUDE_DIRS})

set(${PROJECT_NAME}_SRCS
  src/class_loader.cpp
  src/class_loader_core.cpp
  src/meta_object.cpp
  src/multi_library_class_loader.cpp
)
set(${PROJECT_NAME}_HDRS
  include/class_loader/class_loader.h
  include/class_loader/class_loader_core.h
  include/class_loader/class_loader_exceptions.h
  include/class_loader/class_loader_register_macro.h
  include/class_loader/console_bridge_compatibility.h
  include/class_loader/meta_object.h
  include/class_loader/multi_library_class_loader.h
)
add_definitions(-fPIC)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_HDRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${console_bridge_LIBRARIES} ${Poco_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/class_loader/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/class_loader_headers_update.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(TARGET_NAME ${PROJECT_NAME})
set(PKGCONFIG_LIBS
${Boost_LIBRARIES}
${console_bridge_LIBRARIES}
${Poco_LIBRARIES}
)
message(STATUS "POCO LIBS: ${Poco_LIBRARIES}")
SET(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ CACHE STRING "${PROJECT_NAME} include dirs")
SET(${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME} CACHE STRING "${PROJECT_NAME} libraries")
# Prepare and install necessary files to support finding of the library
# using pkg-config
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/${TARGET_NAME}.pc.in
#  ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
#
  #install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
