cmake_minimum_required(VERSION 2.8.3)
project(nerian_stereo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge
    message_generation dynamic_reconfigure)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(FILES StereoCameraInfo.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs)

# Generate config server C++ headers from the cfg file
generate_dynamic_reconfigure_options(cfg/NerianStereo.cfg)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
include_directories(${CMAKE_CURRENT_LIST_DIR}/cfg/cpp)

# Generate our own run-time parameter update code (from the same cfg file)
message(STATUS "Generate parameter server / dynamic_reconfigure code ...")
execute_process(COMMAND "python" "${CMAKE_CURRENT_LIST_DIR}/cfg/generate_nerian_config_cpp.py"
                "cfg/NerianStereo.cfg"
                "src/autogen_nerian_stereo_dynamic_reconfigure.cpp"
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/"
                RESULT_VARIABLE autogen_result
                OUTPUT_VARIABLE autogen_output
                ERROR_VARIABLE  autogen_output)
if(autogen_result)
    message(STATUS "Output of failing script:\n${autogen_output}")
    message(SEND_ERROR "Failure in code generation script")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  LIBRARIES visiontransfer
  CATKIN_DEPENDS roscpp message_runtime sensor_msgs
#  DEPENDS system_lib
)

#######################
## libvisiontransfer ##
#######################

set(BUILD_VT 1)

if(BUILD_VT)
    # Find lib vision transfer
    execute_process(COMMAND bash -c "ls ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian*.tar.xz | sed -e 's/.*software-\\(.*\\)-src.tar.xz/\\1/' | sort -V | tail -1" OUTPUT_VARIABLE VT_VERSION)
    string(REGEX REPLACE " *\n$" "" VT_VERSION "${VT_VERSION}")

    if(NOT VT_VERSION)
        message(FATAL_ERROR "libvisiontransfer not found!")
    endif()
    message(STATUS "Detected libvisiontransfer version: ${VT_VERSION}")

    # Extract sources while configuring
    execute_process(COMMAND tar --keep-newer-files --warning none -xJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR})

    # Re-extract source files if updated
    add_custom_target(nerian_stereo_untar_src ALL
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz
        COMMAND tar --keep-newer-files --warning none -xJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Check for SSE4.1
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-msse -msse2 -msse3 -msse4.1" SSE4_1_SUPPORT)
    if(SSE4_1_SUPPORT)
        set(VT_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4.1")
    else()
        set(VT_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()

    # Build sources at runtime
    add_custom_target(nerian_stereo_visiontransfer_stub ALL
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz
        COMMAND ${CMAKE_COMMAND} . -DCMAKE_CXX_FLAGS=${VT_CXX_FLAGS} -DDISABLE_NATIVE=1 -DDISABLE_PCL=1 -DDISABLE_OPENCV=1
        COMMAND make
        COMMAND cp lib/libvisiontransfer.so ${CATKIN_DEVEL_PREFIX}/lib/
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer
    )

    install(FILES ${CATKIN_DEVEL_PREFIX}/lib/libvisiontransfer.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer/visiontransfer
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN "CMakeFiles" EXCLUDE
    )

    link_directories(${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer/lib)

    set(COLORCODER_SOURCE_FILE ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/nvcom/helpers/colorcoder.cpp)

else()

    # Development settings: Use local libvisiontransfer
    link_directories($ENV{VISION_TRANSFER_LIB})
    include_directories($ENV{VISION_TRANSFER_INCLUDE})
    set(COLORCODER_SOURCE_FILE $ENV{VISION_TRANSFER_INCLUDE}/../libnvshared/nvshared/imageproc/colorcoder.cpp)
    include_directories($ENV{VISION_TRANSFER_INCLUDE}/../libnvshared/nvshared/imageproc)
endif()

###########
## Build ##
###########

# Activate c++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()	
    message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer
    ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/nvcom/helpers
    ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(nerian_stereo_node
    src/nerian_stereo_node_base.cpp
    src/nerian_stereo_node.cpp
    src/autogen_nerian_stereo_dynamic_reconfigure.cpp
    ${COLORCODER_SOURCE_FILE}
)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(nerian_stereo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    nerian_stereo_visiontransfer_stub ${PROJECT_NAME}_gencfg)

# Specify libraries to link a library or executable target against
target_link_libraries(nerian_stereo_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${OpenCV_LIBS} visiontransfer)

# Do the same things for the nodelet (library) version, too
add_library(nerian_stereo_nodelet
    src/nerian_stereo_node_base.cpp
    src/nerian_stereo_nodelet.cpp
    src/autogen_nerian_stereo_dynamic_reconfigure.cpp
    ${COLORCODER_SOURCE_FILE}
)

add_dependencies(nerian_stereo_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    nerian_stereo_visiontransfer_stub ${PROJECT_NAME}_gencfg)

target_link_libraries(nerian_stereo_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${OpenCV_LIBS} visiontransfer)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS nerian_stereo_node nerian_stereo_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install scripts
install(FILES
    scripts/download_calibration.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
                OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
)

install(FILES data/calib.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(FILES
    launch/nerian_stereo.launch
    launch/nerian_stereo_nodelet.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
