cmake_minimum_required(VERSION 3.1)
project(ixblue_stdbin_decoder
  VERSION 0.1
  LANGUAGES CXX
)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

  # Set default build type to Release if not specified
  set(default_build_type "Release")
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # Compile as C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Let's ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # Testing only available if this is the main app
  option(BUILD_TESTING "" OFF)
  include(CTest)
endif()

## System dependency
find_package(Boost REQUIRED COMPONENTS system)

include(GNUInstallDirs)
if(UNIX)
    option(BUILD_SHARED_LIBS "Build shared libraries(.so)." ON)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
    # for multi-config build system (e.g. Xcode, ninja Multi-Config)
    foreach(OUTPUTCONFIG IN LISTS CMAKE_CONFIGURATION_TYPES)
      string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
      set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/${CMAKE_INSTALL_LIBDIR})
      set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/${CMAKE_INSTALL_LIBDIR})
      set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/${CMAKE_INSTALL_BINDIR})
    endforeach()
else()
  # Currently Only support static build for windows
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
  # for multi-config builds (e.g. msvc)
  foreach(OUTPUTCONFIG IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/${CMAKE_INSTALL_BINDIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/${CMAKE_INSTALL_BINDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/${CMAKE_INSTALL_BINDIR})
  endforeach()
endif()

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED src/memory_block_parser.cpp src/stdbin_decoder.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${Boost_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_auto_type cxx_constexpr cxx_lambdas cxx_nullptr cxx_range_for)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ixblue_stdbin_decoder-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  FILES_MATCHING PATTERN "*.h"
)

# Install catkin package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/ixblue_stdbin_decoder)

install(EXPORT ixblue_stdbin_decoder-targets
  FILE ixblue_stdbin_decoder-targets.cmake
  NAMESPACE ixblue_stdbin_decoder::
  DESTINATION ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/ixblue_stdbin_decoder-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ixblue_stdbin_decoder-config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

write_basic_package_version_file(ixblue_stdbin_decoder-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

#configure_file(cmake/ixblue_stdbin_decoder-config.cmake.in ixblue_stdbin_decoder-config.cmake @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ixblue_stdbin_decoder-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/ixblue_stdbin_decoder-version.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

##########
## Test ##
##########
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  enable_testing()

  ## --- code copied from https://crascit.com/2015/07/25/cmake-gtest/
  # Download and unpack googletest at configure time
  configure_file(cmake/gtest_download.cmake.in googletest-download/CMakeLists.txt)
  execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download" )
  execute_process(COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download" )

  # Add googletest directly to our build. This adds
  # the following targets: gtest, gtest_main, gmock
  # and gmock_main
  add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
  "${CMAKE_BINARY_DIR}/googletest-build")

  # The gtest/gmock targets carry header search path
  # dependencies automatically when using CMake 2.8.11 or
  # later. Otherwise we have to add them here ourselves.
  if(CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include")
  endif()

  ## ---

  ## Add gtest based cpp test target and link libraries
  add_executable(memory_blocs_parser_navigation-test test/memory_blocs_parser_navigation.test.cc)
  target_link_libraries(memory_blocs_parser_navigation-test ${PROJECT_NAME} ${Boost_LIBRARIES} gtest)
  set_target_properties(memory_blocs_parser_navigation-test PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  add_test(NAME memory_blocs_parser_navigation_test COMMAND memory_blocs_parser_navigation-test WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)

  add_executable(memory_blocs_parser_extended_navigation-test test/memory_blocs_parser_extended_navigation.test.cc)
  target_link_libraries(memory_blocs_parser_extended_navigation-test ${PROJECT_NAME} ${Boost_LIBRARIES} gtest)
  set_target_properties(memory_blocs_parser_extended_navigation-test PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  add_test(NAME memory_blocs_parser_extended_navigation-test COMMAND memory_blocs_parser_extended_navigation-test WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)

  add_executable(memory_blocs_parser_external_data-test test/memory_blocs_parser_external_data.test.cc)
  target_link_libraries(memory_blocs_parser_external_data-test ${PROJECT_NAME} ${Boost_LIBRARIES} gtest)
  set_target_properties(memory_blocs_parser_external_data-test PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  add_test(NAME memory_blocs_parser_external_data-test COMMAND memory_blocs_parser_external_data-test WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)

  add_executable(stdbin_decoder-test test/stdbin_decoder.test.cc)
  target_link_libraries(stdbin_decoder-test ${PROJECT_NAME} ${Boost_LIBRARIES} gtest)
  set_target_properties(stdbin_decoder-test PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  add_test(NAME stdbin_decoder-test COMMAND stdbin_decoder-test WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)

endif()
