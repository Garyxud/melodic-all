cmake_minimum_required(VERSION 2.6)
project(SDHLibrary-CPP)

## common source files
set(lib_sources
    "sdh/crc.cpp"
    "sdh/dsa.cpp"
    "sdh/rs232-cygwin.cpp"
    "sdh/sdhbase.cpp"
    "sdh/sdh_codes.cpp"
    "sdh/sdh.cpp"
    "sdh/sdhexception.cpp"
    "sdh/sdhserial.cpp"
    "sdh/serialbase.cpp"
    "sdh/simplestringlist.cpp"
    "sdh/simplevector.cpp"
    "sdh/tcpserial.cpp"
    "sdh/unit_converter.cpp"
    "sdh/util.cpp"
)

## common header files
set(lib_header
    "sdh/basisdef.h"
    "sdh/crc.h"
    "sdh/dbg.h"
    "sdh/dsa.h"
    "sdh/release.h"
    "sdh/rs232-cygwin.h"
    "sdh/sdhbase.h"
    "sdh/sdh_codes.h"
    "sdh/sdh_command_codes.h"
    "sdh/sdhexception.h"
    "sdh/sdh.h"
    "sdh/sdhlibrary_settings.h"
    "sdh/sdh_return_codes.h"
    "sdh/sdhserial.h"
    "sdh/serialbase.h"
    "sdh/simplestringlist.h"
    "sdh/simpletime.h"
    "sdh/simplevector.h"
    "sdh/tcpserial.h"
    "sdh/unit_converter.h"
    "sdh/util.h"
)

## add source and header files depending on operation system and CAN support

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    add_definitions(-DOSNAME_LINUX=1)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Cygwin")
    add_definitions(-DOSNAME_CYGWIN=1)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    list(APPEND lib_sources "sdh/rs232-vcc.cpp")
    list(APPEND lib_header "sdh/rs232-vcc.h")
endif()

option(WITH_ESD_CAN "ESD CAN support" OFF)
option(WITH_PEAK_CAN "PEAK CAN support" OFF)

if(WITH_ESD_CAN)
    find_package(libntcan REQUIRED)
    include_directories(${libntcan_INCLUDE_DIRS}/libntcan)
    link_libraries(${libntcan_LIBRARIES})
    add_definitions(-DWITH_ESD_CAN=1)
    list(APPEND lib_sources "sdh/canserial-esd.cpp")
    list(APPEND lib_header "sdh/canserial-esd.h")
else()
    add_definitions(-DWITH_ESD_CAN=0)
endif()

if(WITH_PEAK_CAN)
    find_package(libpcan REQUIRED)
    include_directories(${libpcan_INCLUDE_DIRS}/libpcan)
    link_libraries(${libpcan_LIBRARIES})
    add_definitions(-DWITH_PEAK_CAN=1)
    list(APPEND lib_sources "sdh/canserial-peak.cpp")
    list(APPEND lib_header "sdh/canserial-peak.h")
else()
    add_definitions(-DWITH_PEAK_CAN=0)
endif()

## define library
add_library(${PROJECT_NAME} SHARED ${lib_sources})

## install library
include(GNUInstallDirs)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${lib_header}")
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdh>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
install(TARGETS ${PROJECT_NAME} EXPORT sdhlibrary_cpp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/schunk_sdh
)

install(EXPORT sdhlibrary_cpp
        DESTINATION share/sdhlibrary_cpp/cmake
        FILE sdhlibrary_cppConfig.cmake
)

option(BUILD_DEMOS "Build demo programs" OFF)

if(BUILD_DEMOS)
    ## demo source files
    set(demo_sources
        "demo/demo-benchmark.cpp"
        "demo/demo-dsa.cpp"
        "demo/demo-dsa-simple.cpp"
        "demo/demo-GetAxisActualAngle.cpp"
        "demo/demo-GetFingerXYZ.cpp"
        "demo/demo-griphand.cpp"
        "demo/demo-mimic.cpp"
        "demo/demo-radians.cpp"
        "demo/demo-simple2.cpp"
        "demo/demo-simple3.cpp"
        "demo/demo-simple.cpp"
        "demo/demo-simple-withtiming.cpp"
        "demo/demo-temperature.cpp"
        "demo/demo-velocity-acceleration.cpp"
    )

    ## demo executables
    include_directories(.)
    foreach(demo_source IN LISTS demo_sources)
        get_filename_component(demo_name ${demo_source} NAME_WE)
        add_executable(${demo_name} ${demo_source} "demo/sdhoptions.cpp")
        target_link_libraries(${demo_name} ${PROJECT_NAME})
        install(TARGETS ${demo_name} RUNTIME DESTINATION bin)
    endforeach()

    if(WITH_ESD_CAN OR WITH_PEAK_CAN)
        add_executable(cancat demo/cancat.cpp "demo/sdhoptions.cpp")
        target_link_libraries(cancat ${PROJECT_NAME})
        install(TARGETS cancat RUNTIME DESTINATION bin)
    endif()

    find_package(Boost COMPONENTS system thread)
    if(${Boost_FOUND})
        include_directories(${Boost_INCLUDE_DIRS})
        link_libraries(${Boost_LIBRARIES})
        add_executable(demo-contact-grasping demo/demo-contact-grasping.cpp demo/sdhoptions.cpp demo/dsaboost.cpp)
        target_link_libraries(demo-contact-grasping ${PROJECT_NAME})
        install(TARGETS demo-contact-grasping RUNTIME DESTINATION bin)
    endif()
endif()
