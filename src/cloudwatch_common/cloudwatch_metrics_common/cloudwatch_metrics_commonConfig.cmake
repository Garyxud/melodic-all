# Compute paths
set(cloudwatch_metrics_common_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")

if(NOT TARGET cloudwatch_metrics_common)
  include("${CMAKE_CURRENT_LIST_DIR}/cloudwatch_metrics_common-targets.cmake")
endif()

set(cloudwatch_metrics_common_LIBRARIES cloudwatch_metrics_common)

# where the .pc pkgconfig files are installed
set(cloudwatch_metrics_common_PKGCONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/pkgconfig")
