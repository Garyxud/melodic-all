cmake_minimum_required(VERSION 2.8.3)
project(cloudwatch_metrics_common)
set(CLOUDWATCH_METRICS_COMMON_VERSION 1.1.0)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
set(FLAGS -pedantic -Wno-long-long -Wall -Wextra -Wcast-qual -Wformat -Wwrite-strings -Wcast-align -Wno-error=cast-align)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

find_package(dataflow_lite REQUIRED)
find_package(file_management REQUIRED)
find_package(aws_common REQUIRED)

if(AWSSDK_FOUND)
  set(SERVICE monitoring)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE OUTPUT)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

set(ALL_SRC_FILES
      src/utils/cloudwatch_metrics_facade.cpp
      src/metric_batcher.cpp
      src/metric_publisher.cpp
      src/metric_service_factory.cpp
      src/utils/metric_file_manager.cpp
      src/utils/metric_serialization.cpp
      )

add_library(${PROJECT_NAME} ${ALL_SRC_FILES})
target_link_libraries(${PROJECT_NAME}
        ${OUTPUT}
        ${file_management_LIBRARIES}
        ${dataflow_lite_LIBRARIES} )

target_include_directories(${PROJECT_NAME} PRIVATE
        ${aws_common_INCLUDE_DIRS}
        ${AWSSDK_INCLUDE_DIR}
        ${file_management_INCLUDE_DIRS}
        ${dataflow_lite_INCLUDE_DIRS} )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}-targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include/
)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(${PROJECT_NAME}Config.cmake.in
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(${PROJECT_NAME}ConfigVersion.cmake.in
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)


#############
## Testing ##
#############

enable_testing()
find_common_test_packages()
if(GMOCK_LIBRARY)

  macro(link_test_target target_name)
    if(TARGET ${target_name})
      target_include_directories("${target_name}" PRIVATE ${HEADERS_FOR_TESTS})
      target_link_libraries("${target_name}" ${LIBS_FOR_TESTS})
    endif()
  endmacro()

  add_common_gtest(test_cloudwatch_metrics
          test/main_test.cpp
          test/metric_publisher_test.cpp
          test/metric_serialization_test.cpp
          )

  add_common_gtest(test_pipelines
          test/metric_pipeline_test.cpp
          )
  set(LIBS_FOR_TESTS
          ${GTEST_LIBRARIES}
          pthread
          ${GMOCK_LIBRARY}
          ${PROJECT_NAME}
          )

  set(HEADERS_FOR_TESTS
          PRIVATE include
          PRIVATE ${aws_common_INCLUDE_DIRS}
          PUBLIC ${PROJECT_SOURCE_DIR}/test/include
          )

  link_test_target(test_cloudwatch_metrics)
  link_test_target(test_pipelines)

endif()
