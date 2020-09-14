cmake_minimum_required (VERSION 3.5.1)
project (cpplogging C CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)
#It seams that from Ubuntu 17.10 the linker flag --disable-new-dtags is needed
#for the executable to be able to find absolute linked shared libraries with rpath.
#https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")

# Remove 'lib' prefix for shared libraries on Windows
if (WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif ()

SET(PROJECT_NAME_ORIG cpplogging)

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

###############################
###############################

add_library(${PROJECT_NAME} SHARED
    src/Loggable.cpp
    src/Logger.cpp
    )

SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")


SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include
    )

message(STATUS "${PROJECT_NAME} include dirs: ${${PROJECT_NAME}_INCLUDE_DIRS}")

target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} pthread)

#https://cmake.org/cmake/help/v3.0/command/install.html 
install(TARGETS ${PROJECT_NAME}	LIBRARY DESTINATION "lib")
install(DIRECTORY "spdlog/include/spdlog" DESTINATION "include")
install(DIRECTORY "includes/cpplogging" DESTINATION "include")

SET(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS} CACHE STRING "${PROJECT_NAME} include dirs")
SET(${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME} CACHE STRING "${PROJECT_NAME} libraries")

message("End ${PROJECT_NAME_ORIG}")
