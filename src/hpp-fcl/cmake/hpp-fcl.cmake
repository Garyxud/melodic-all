# The headers are in hpp/fcl/. This project makes this hpp-fcl/hpp/fcl.
# We still need to expose "hpp/fcl/" as the include path:
set(hpp-fcl_INCLUDE_DIRS "${hpp-fcl_INCLUDE_DIRS}/hpp-fcl")
include_directories(${hpp-fcl_INCLUDE_DIRS})

# Export definitions that are required
find_package(assimp REQUIRED)
if(assimp_FOUND)
  if (NOT ${assimp_VERSION} VERSION_LESS "2.0.1150")
    add_definitions(-DFCL_USE_ASSIMP_UNIFIED_HEADER_NAMES)
    SET(WITH_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES TRUE)
    message(STATUS "Assimp version has unified headers")
  else()
    SET(WITH_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES FALSE)
    message(STATUS "Assimp version does not have unified headers")
  endif()
else()
  message(FATAL_ERROR "Assimp not found ${assimp_FOUND}")
endif()

find_package(octomap REQUIRED)
if(octomap_FOUND)
    add_definitions(-DFCL_HAVE_OCTOMAP -DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
else()
    message(FATAL_ERROR "Octomap not found")
endif()
