cmake_minimum_required(VERSION 2.8.3)
project(libreflexxestype2)

find_package(catkin REQUIRED)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

find_program(CMAKE_DPKG dpkg /bin /usr/bin /usr/local/bin) 
if(CMAKE_DPKG)
  exec_program(dpkg ARGS --print-architecture OUTPUT_VARIABLE CMAKE_DPKG_ARCH)
  if(CMAKE_DPKG_ARCH MATCHES "amd64")
    message(STATUS "+++++++++++++++++ DETECTED 64 bit +++++++++++++++++")
    set(SUPPORTED "true")
#   elseif(CMAKE_DPKG_ARCH MATCHES "i386")
#     message(STATUS "+++++++++++++++++ DETECTED 32 bit +++++++++++++++++")
#     set(SUPPORTED "true")
  else()
    message(WARNING "+++++++++++++++++ UNSUPPORTED ARCHITECTURE, UNABLE TO USE ReflexxesTypeII +++++++++++++++++")
    hset(SUPPORTED "false")
  endif()
endif()

if(SUPPORTED MATCHES "true")
  catkin_package(
    CFG_EXTRAS libreflexxestypeII-extras.cmake # catkin_package does not support artifacts in devel space
    INCLUDE_DIRS include 
  )
  
  
  # Concorde expects qsopt.a to be in the same directory as qsopt.h --> static library, when using in another package one has to provide an absolute path to it
  add_custom_target(
    build_reflexxes ALL
        # copy headers to devel space (catkin does not like headers in source space)
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
        # copy libs, set-up soname chain
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
        COMMAND ${CMAKE_COMMAND} -E create_symlink libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libreflexxestypeII.so
  )

  install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libReflexxesTypeII.so
                   ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libreflexxestypeII.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

endif()
