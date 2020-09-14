# Compute paths
set(dataflow_lite_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")

if(NOT TARGET dataflow_lite)
    include("${CMAKE_CURRENT_LIST_DIR}/dataflow_lite-targets.cmake")
endif()

set(dataflow_lite_LIBRARIES dataflow_lite)

# where the .pc pkgconfig files are installed
set(dataflow_lite_PKGCONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/pkgconfig")