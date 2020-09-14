# This file is part of the rc_dynamics_api package.
#
# Copyright (c) 2017 Roboception GmbH
# All rights reserved
#
# Author: Christian Emmerich
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

project(rc_dynamics_api CXX)

## Adding cpr and protobuf libraries
####################################

add_subdirectory(opt)
include_directories(${CPR_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})

## Compiling and building protobuf messages into a library
##########################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/Modules/")
include(ProtobufGenerate)

# let shared objects include static libs
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# set correct include dirs
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_BINARY_DIR})

# List of absolute file names to each proto file
SET(PROTOS)

SET(RC_DYNAMICS_MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rc_dynamics_msgs")

# find all proto files relative to this source dir
FILE(GLOB_RECURSE PROTOS_REL RELATIVE "${RC_DYNAMICS_MSGS_DIR}"
        "${RC_DYNAMICS_MSGS_DIR}/roboception/msgs/[^._]*.proto")

# find all proto files, store them in a list for later processing, and copy them to the binary dir
FOREACH(FILE ${PROTOS_REL})
    SET(OUT "${CMAKE_CURRENT_BINARY_DIR}/${FILE}")
    ADD_CUSTOM_COMMAND(
            OUTPUT ${OUT}
            COMMAND ${CMAKE_COMMAND} -E copy "${RC_DYNAMICS_MSGS_DIR}/${FILE}" "${OUT}"
            DEPENDS "${RC_DYNAMICS_MSGS_DIR}/${FILE}"
            COMMENT "Copying proto file ${FILE} to binary dir")
    LIST(APPEND PROTOS "${OUT}")
ENDFOREACH()

ADD_CUSTOM_TARGET("protos" DEPENDS ${PROTOS} )

# define the root, now in the build tree, for later processing
set(PROTO_ROOT "${CMAKE_CURRENT_BINARY_DIR}")

# generate the cpp files from protos
PROTOBUF_GENERATE(CPP PROTO_SRC PROTO_HEADER
        PROTOFILES ${PROTOS}
        PROTOROOT ${PROTO_ROOT}
        ${INCLUDE_ARG}
        DEPENDS "protos" ${UPSTREAM_TARGETS}
        OUTPATH ${CMAKE_CURRENT_BINARY_DIR})

ADD_LIBRARY(protolib STATIC ${PROTO_SRC} ${PROTO_HEADER})
TARGET_LINK_LIBRARIES(protolib ${PROTOBUF_LIBRARY} ${PROTOBUF_LIBRARIES})

## Adding own library for pose interface
########################################

set(src
    net_utils.cc
    remote_interface.cc
    socket_exception.cc
    unexpected_receive_timeout.cc
    trajectory_time.cc
    ${CMAKE_CURRENT_BINARY_DIR}/project_version.cc
)

set(hh
    net_utils.h
    remote_interface.h
    data_receiver.h
    msg_utils.h
    socket_exception.h
    unexpected_receive_timeout.h
    trajectory_time.h
    ${CMAKE_CURRENT_BINARY_DIR}/project_version.h
)

add_library(rc_dynamics_api_static STATIC ${src})
target_link_libraries(rc_dynamics_api_static ${CPR_LIBRARIES} protolib)

# install(TARGETS rc_dynamics_api_static EXPORT PROJECTTargets COMPONENT dev DESTINATION lib)

if (BUILD_SHARED_LIBS)
    add_library(rc_dynamics_api SHARED ${src})
    target_link_libraries(rc_dynamics_api LINK_PRIVATE ${CPR_LIBRARIES} protolib)
    set_target_properties(rc_dynamics_api PROPERTIES SOVERSION ${abiversion})

    install(TARGETS rc_dynamics_api EXPORT PROJECTTargets COMPONENT bin
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
endif ()

# only install headers if we build and install the shared lib
if (BUILD_SHARED_LIBS)
    install(FILES ${hh} COMPONENT dev DESTINATION include/rc_dynamics_api)
    install(FILES ${PROTO_HEADER} COMPONENT dev DESTINATION include/roboception/msgs)
endif ()
