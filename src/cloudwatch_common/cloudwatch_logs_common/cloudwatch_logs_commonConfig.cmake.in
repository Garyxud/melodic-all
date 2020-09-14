# Compute paths
set(cloudwatch_logs_common_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")

if(NOT TARGET cloudwatch_logs_common)
    include("${CMAKE_CURRENT_LIST_DIR}/cloudwatch_logs_common-targets.cmake")
endif()

set(cloudwatch_logs_common_LIBRARIES cloudwatch_logs_common)

# where the .pc pkgconfig files are installed
set(cloudwatch_logs_common_PKGCONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/pkgconfig")