# Locate and configure the Google Protocol Buffers library.
# A modified version of the original macro from CMake 2.8.
# Defines the following variables:
#
#   PROTOBUF_FOUND - Found the Google Protocol Buffers library
#   PROTOBUF_INCLUDE_DIRS - Include directories for Google Protocol Buffers
#   PROTOBUF_LIBRARIES - The protobuf library
#
# The following cache variables are also defined:
#   PROTOBUF_LIBRARY - The protobuf library
#   PROTOBUF_PROTOC_LIBRARY   - The protoc library
#   PROTOBUF_INCLUDE_DIR - The include directory for protocol buffers
#   PROTOBUF_PROTOC_EXECUTABLE - The protoc compiler
#
# These variables are read for additional hints:
#   PROTOBUF_ROOT - Root directory of the protobuf installation if not found
#                   automatically
#
#  ====================================================================
#  Example:
#
#   find_package(ProtocolBuffers REQUIRED)
#   include_directories(${PROTOBUF_INCLUDE_DIRS})
#
#   include_directories(${CMAKE_CURRENT_BINARY_DIR})
#   PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS foo.proto)
#   add_executable(bar bar.cc ${PROTO_SRCS} ${PROTO_HDRS})
#   target_link_libraries(bar ${PROTOBUF_LIBRARY})
#
# NOTE: You may need to link against pthreads, depending
# on the platform.
#  ====================================================================

#=============================================================================
# Copyright 2009 Kitware, Inc.
# Copyright 2009 Philip Lowman <philip@yhbt.com>
# Copyright 2008 Esben Mose Hansen, Ange Optimization ApS
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distributed this file outside of CMake, substitute the full
#  License text for the above reference.)

FIND_PATH(PROTOBUF_INCLUDE_DIR NAMES google/protobuf/service.h
          HINTS "${PROTOBUF_ROOT}/include"
          DOC "The Google Protocol Buffers Headers")

# Google's provided vcproj files generate libraries with a "lib"
# prefix on Windows
IF(WIN32)
    SET(PROTOBUF_ORIG_FIND_LIBRARY_PREFIXES "${CMAKE_FIND_LIBRARY_PREFIXES}")
    SET(CMAKE_FIND_LIBRARY_PREFIXES "lib" "")
ENDIF()

FIND_LIBRARY(PROTOBUF_LIBRARY NAMES protobuf
             HINTS "${PROTOBUF_ROOT}/bin"
                   "${PROTOBUF_ROOT}/lib"
             DOC "The Google Protocol Buffers Library"
)
FIND_LIBRARY(PROTOBUF_PROTOC_LIBRARY NAMES protoc
             HINTS "${PROTOBUF_ROOT}/bin"
                   "${PROTOBUF_ROOT}/lib"
             DOC "The Google Protocol Buffers Compiler Library"
)
FIND_PROGRAM(PROTOBUF_PROTOC_EXECUTABLE NAMES protoc
             HINTS "${PROTOBUF_ROOT}/bin"
             DOC "The Google Protocol Buffers Compiler"
)
IF(PROTOBUF_PROTOC_EXECUTABLE AND NOT PROTOBUF_PROTOC_MATLAB)
    # check whether this protoc version supports matlab
    EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} "-h"
                    ERROR_VARIABLE PROTOC_HELP_TEXT
                    OUTPUT_QUIET)
    STRING(REGEX MATCH "--matlab_out" PROTOC_MATLAB_OUT "${PROTOC_HELP_TEXT}")
    IF(PROTOC_MATLAB_OUT)
        SET(PROTOBUF_PROTOC_MATLAB TRUE CACHE BOOL "Whether protoc is able to generate matlab output.")
        MESSAGE(STATUS "protoc supports matlab")
    ELSE()
        SET(PROTOBUF_PROTOC_MATLAB FALSE CACHE BOOL "Whether protoc is able to generate matlab output.")
        MESSAGE(STATUS "protoc does not support matlab")
    ENDIF()
ENDIF()
SET(PROTOBUF_PROTOC_VERSION "PROTOBUF_PROTOC_VERSION-NOTFOUND")
IF(PROTOBUF_PROTOC_EXECUTABLE)
    EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --version
                    OUTPUT_VARIABLE PROTOBUF_PROTOC_VERSION_TEMP
                    RESULT_VARIABLE PROTOBUF_PROTOC_VERSION_RESULT)
    STRING(REGEX REPLACE ".*([0-9]\\.[0-9]\\.[0-9]).*" "\\1"
           PROTOBUF_PROTOC_VERSION_TEMP "${PROTOBUF_PROTOC_VERSION_TEMP}")
    IF(PROTOBUF_PROTOC_VERSION_TEMP)
        SET(PROTOBUF_PROTOC_VERSION "${PROTOBUF_PROTOC_VERSION_TEMP}")
    ENDIF()
ENDIF()

SET(KNOWN_VERSIONS 2.6.0 2.5.1 2.5.0 2.4.1 2.4.0 2.3.0)
# if we know the compiler version, we should favor it
IF(PROTOBUF_PROTOC_VERSION)
    LIST(INSERT KNOWN_VERSIONS 0 ${PROTOBUF_PROTOC_VERSION})
ENDIF()
SET(JAVA_NAMES ${PROTOBUF_JAVA_NAME} protobuf.jar protobuf-java.jar)
FOREACH(VERSION ${KNOWN_VERSIONS})
    LIST(APPEND JAVA_NAMES "protobuf-java-${VERSION}.jar")
ENDFOREACH()

FIND_FILE(PROTOBUF_JAVA_LIBRARY
          NAMES ${JAVA_NAMES}
          HINTS ${PROTOBUF_JAVA_ROOT}
                "${PROTOBUF_JAVA_ROOT}/share/java"
                "/usr/share/java"
                "/usr/share/java/protobuf-java"
                "/usr/share/protobuf/lib"
                "${CMAKE_INSTALL_PREFIX}/lib/java"
                "${CMAKE_INSTALL_PREFIX}/share/java")

MARK_AS_ADVANCED(PROTOBUF_INCLUDE_DIR
                 PROTOBUF_LIBRARY
                 PROTOBUF_PROTOC_LIBRARY
                 PROTOBUF_PROTOC_EXECUTABLE
                 PROTOBUF_JAVA_LIBRARY
                 PROTOBUF_PROTOC_MATLAB)

# Restore original find library prefixes
IF(WIN32)
    SET(CMAKE_FIND_LIBRARY_PREFIXES "${PROTOBUF_ORIG_FIND_LIBRARY_PREFIXES}")
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PROTOBUF DEFAULT_MSG PROTOBUF_LIBRARY PROTOBUF_INCLUDE_DIR)

IF(PROTOBUF_FOUND)
    SET(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIR})
    SET(PROTOBUF_LIBRARIES    ${PROTOBUF_LIBRARY})
ENDIF()
