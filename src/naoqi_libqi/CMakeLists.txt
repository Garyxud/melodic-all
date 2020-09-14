## Copyright (c) 2012, 2013, 2014 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

cmake_minimum_required(VERSION 2.8)
project(naoqi_libqi)
find_package(catkin)
find_package(OpenSSL REQUIRED)

catkin_package(CFG_EXTRAS libqi-extras.cmake
   INCLUDE_DIRS ./
   LIBRARIES qi
)

include(CheckSymbolExists)
check_symbol_exists(_SC_HOST_NAME_MAX "unistd.h" HAVE_SC_HOST_NAME_MAX)
set_source_files_properties(src/os_posix.cpp
  PROPERTIES
    COMPILE_DEFINITIONS HAVE_SC_HOST_NAME_MAX)

# We always want boost filesystem v3
add_definitions("-DBOOST_FILESYSTEM_VERSION=3")
add_definitions("-DBOOST_ASIO_DISABLE_STD_CHRONO")

if(MSVC)
  add_definitions(" -DWIN32_LEAN_AND_MEAN -DNOMINMAX")
endif()

if (WIN32)
  add_definitions(" -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DWIN32_LEAN_AND_MEAN -DNOMINMAX")
endif()

if (UNIX)
 add_definitions(" -fno-strict-aliasing ")
endif()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

# Remove this line to use QT if usable
set(WITH_QT5_CORE OFF)

if(WITH_BOOST_LOCALE)
  add_definitions(" -DWITH_BOOST_LOCALE ")
endif()

enable_testing()
include(CMakeDependentOption)

if (WITH_PROBES)
  find_package(qiprobes)
  qiprobes_create_probe(tp_qi
    src/tp_qi.in.h
    PROVIDER_NAME qi_qi)
  qiprobes_instrument_files(tp_qi
    src/eventloop.cpp)
  set(_tp_qi "tp_qi")
else()
  set(_tp_qi "")
endif()

set(QI_H qi/detail/async.hxx
         qi/detail/clock.hxx
         qi/detail/eventloop.hxx
         qi/detail/future.hxx
         qi/detail/future_fwd.hpp
         qi/detail/futurebarrier.hpp
         qi/detail/futureunwrap.hpp
         qi/detail/executioncontext.hpp
         qi/detail/log.hxx
         qi/detail/mpl.hpp
         qi/detail/print.hpp
         qi/detail/trackable.hxx
         qi/detail/warn_push_ignore_deprecated.hpp
         qi/detail/warn_pop_ignore_deprecated.hpp
         qi/api.hpp
         qi/application.hpp
         qi/actor.hpp
         qi/async.hpp
         qi/atomic.hpp
         qi/buffer.hpp
         qi/clock.hpp
         qi/either.hpp
         qi/flags.hpp
         qi/future.hpp
         qi/futuregroup.hpp
         qi/log/consoleloghandler.hpp
         qi/log/csvloghandler.hpp
         qi/log/fileloghandler.hpp
         qi/log/headfileloghandler.hpp
         qi/log/tailfileloghandler.hpp
         qi/log.hpp
         qi/macro.hpp
         qi/numeric.hpp
         qi/os.hpp
         qi/getenv.hpp
         qi/preproc.hpp
         qi/qi.hpp
         qi/types.hpp
         qi/path.hpp
         qi/path_conf.hpp
         qi/periodictask.hpp
         qi/qi.hpp
         qi/stats.hpp
         qi/trackable.hpp
         qi/translator.hpp
         qi/eventloop.hpp
         qi/version.hpp
         qi/iocolor.hpp
         qi/strand.hpp)

set(QI_C src/dlfcn.cpp
         src/path.cpp
         src/application.cpp
         src/buffer.cpp
         src/buffer_p.hpp
         src/bufferreader.cpp
         src/clock.cpp
         src/sdklayout.hpp
         src/future.cpp
         src/log.cpp
         src/log_p.hpp
         src/consoleloghandler.cpp
         src/fileloghandler.cpp
         src/csvloghandler.cpp
         src/headfileloghandler.cpp
         src/tailfileloghandler.cpp
         src/locale-light.cpp
         src/os.cpp
         src/path_conf.cpp
         src/periodictask.cpp
         src/print.cpp
         src/utils.hpp
         src/utils.cpp
         src/eventloop.cpp
         src/eventloop_p.hpp
         src/sdklayout-boost.cpp
         src/version.cpp
         src/iocolor.cpp
         src/strand.cpp
         src/ptruid.cpp
         qi/assert.hpp

         qi/ptruid.hpp
         qi/objectuid.hpp
         qi/tag.hpp
         qi/uuid.hpp)

set(QITYPE_H       qi/anyfunction.hpp
                   qi/anyobject.hpp
                   qi/signature.hpp
                   qi/property.hpp
                   qi/signal.hpp
                   qi/signalspy.hpp
                   qi/anyvalue.hpp
                   qi/anymodule.hpp

                   qi/type/detail/signal.hxx
                   qi/type/detail/property.hxx
                   qi/type/detail/accessor.hxx
                   qi/type/detail/anyreference.hpp
                   qi/type/detail/anyreference.hxx
                   qi/type/detail/anyvalue.hpp
                   qi/type/detail/anyvalue.hxx
                   qi/type/detail/anyfunction.hxx
                   qi/type/detail/anyfunctionfactory.hxx
                   qi/type/detail/anyiterator.hpp
                   qi/type/detail/anyiterator.hxx
                   qi/type/detail/bindtype.hxx
                   qi/type/detail/functionsignature.hxx
                   qi/type/detail/futureadapter.hxx
                   qi/type/detail/futureadapter.hpp
                   qi/type/detail/dynamicobjectbuilder.hxx
                   qi/type/detail/genericobject.hpp
                   qi/type/detail/object.hxx
                   qi/type/detail/proxyregister.hpp
                   qi/type/detail/hasless.hxx
                   qi/type/detail/objecttypebuilder.hxx
                   qi/type/detail/type.hxx
                   qi/type/detail/buffertypeinterface.hxx
                   qi/type/detail/typedispatcher.hxx
                   qi/type/detail/dynamictypeinterface.hxx
                   qi/type/detail/typeimpl.hxx
                   qi/type/detail/typeinterface.hpp
                   qi/type/detail/inttypeinterface.hxx
                   qi/type/detail/listtypeinterface.hxx
                   qi/type/detail/maptypeinterface.hxx
                   qi/type/detail/optionaltypeinterface.hxx
                   qi/type/detail/pointertypeinterface.hxx
                   qi/type/detail/staticobjecttype.hpp
                   qi/type/detail/stringtypeinterface.hxx
                   qi/type/detail/structtypeinterface.hxx
                   qi/type/detail/type.hpp
                   qi/type/detail/manageable.hpp
                   qi/type/detail/traceanalyzer.hpp

                   qi/api.hpp
                   qi/binarycodec.hpp
                   qi/type/dynamicobject.hpp
                   qi/type/dynamicobjectbuilder.hpp
                   qi/type/fwd.hpp
                   qi/jsoncodec.hpp
                   qi/type/metamethod.hpp
                   qi/type/metaobject.hpp
                   qi/type/metaproperty.hpp
                   qi/type/metasignal.hpp
                   qi/type/objecttypebuilder.hpp
                   qi/type/proxyproperty.hpp
                   qi/type/proxysignal.hpp
                   qi/type/typeinterface.hpp
                   qi/type/typeobject.hpp
                   qi/type/typedispatcher.hpp)


set(QITYPE_C src/type/binarycodec.cpp
             src/type/binarycodec_p.hpp
             src/type/dynamicobject.cpp
             src/type/dynamicobjectbuilder.cpp
             src/type/anyfunction.cpp
             src/type/anyreference.cpp
             src/type/anyvalue.cpp
             src/type/anyobject.cpp
             src/type/genericobject.cpp
             src/type/jsoncodec_p.hpp
             src/type/jsondecoder.cpp
             src/type/jsonencoder.cpp
             src/type/manageable.cpp
             src/type/metamethod.cpp
             src/type/metaproperty.cpp
             src/type/metasignal.cpp
             src/type/metasignal_p.cpp
             src/type/metasignal_p.hpp
             src/type/metaobject.cpp
             src/type/metaobject_p.hpp
             src/type/anymodule.cpp
             src/type/objecttypebuilder.cpp
             src/type/signal.cpp
             src/type/signal_p.hpp
             src/type/signalspy.cpp
             src/type/signatureconvertor.cpp
             src/type/signatureconvertor.hpp
             src/type/staticobjecttype.cpp
             src/type/typeinterface.cpp
             src/type/structtypeinterface.cpp
             src/type/type.cpp
             src/type/signature.cpp
             src/type/traceanalyzer.cpp
             )


set(QIM_H qi/api.hpp
          qi/messaging/authprovider.hpp
          qi/messaging/authproviderfactory.hpp
          qi/messaging/autoservice.hpp
          qi/messaging/clientauthenticator.hpp
          qi/messaging/clientauthenticatorfactory.hpp
          qi/messaging/detail/autoservice.hxx
          qi/messaging/gateway.hpp
          qi/messaging/messagesocket_fwd.hpp
          qi/messaging/servicedirectoryproxy.hpp
          qi/messaging/serviceinfo.hpp
          qi/applicationsession.hpp
          qi/session.hpp
          qi/url.hpp
)

set(QIM_C
  src/messaging/applicationsession_internal.hpp
  src/messaging/applicationsession_internal.cpp
  src/messaging/applicationsession.cpp
  src/messaging/authprovider_p.hpp
  src/messaging/authprovider.cpp
  src/messaging/boundobject.cpp
  src/messaging/boundobject.hpp
  src/messaging/clientauthenticator_p.hpp
  src/messaging/clientauthenticator.cpp
  src/messaging/gateway.cpp
  src/messaging/message.hpp
  src/messaging/message.cpp
  src/messaging/messagedispatcher.hpp
  src/messaging/messagedispatcher.cpp
  src/messaging/objecthost.hpp
  src/messaging/objecthost.cpp
  src/messaging/objectregistrar.hpp
  src/messaging/objectregistrar.cpp
  src/messaging/remoteobject.cpp
  src/messaging/remoteobject_p.hpp
  src/messaging/servicedirectory.cpp
  src/messaging/servicedirectory.hpp
  src/messaging/servicedirectoryclient.hpp
  src/messaging/servicedirectoryclient.cpp
  src/messaging/servicedirectoryproxy.cpp
  src/messaging/serviceinfo.cpp
  src/messaging/session.cpp
  src/messaging/session_p.hpp
  src/messaging/sessionservice.hpp
  src/messaging/sessionservice.cpp
  src/messaging/sessionservices.hpp
  src/messaging/sessionservices.cpp
  src/messaging/server.hpp
  src/messaging/server.cpp
  src/messaging/streamcontext.hpp
  src/messaging/streamcontext.cpp
  src/messaging/transportserver.hpp
  src/messaging/transportserver.cpp
  src/messaging/transportserverasio_p.cpp
  src/messaging/transportserverasio_p.hpp
  src/messaging/messagesocket.hpp
  src/messaging/messagesocket.cpp
  src/messaging/transportsocketcache.cpp
  src/messaging/transportsocketcache.hpp
  src/messaging/tcpmessagesocket.cpp
  src/messaging/tcpmessagesocket.hpp
  src/messaging/url.cpp
  src/registration.cpp
)

set(QI_SOCKET_H
  src/messaging/sock/accept.hpp
  src/messaging/sock/common.hpp
  src/messaging/sock/concept.hpp
  src/messaging/sock/connect.hpp
  src/messaging/sock/connectedstate.hpp
  src/messaging/sock/connectingstate.hpp
  src/messaging/sock/disconnectedstate.hpp
  src/messaging/sock/disconnectingstate.hpp
  src/messaging/sock/error.hpp
  src/messaging/sock/macrolog.hpp
  src/messaging/sock/socketptr.hpp
  src/messaging/sock/sslcontextptr.hpp
  src/messaging/sock/socketwithcontext.hpp
  src/messaging/sock/networkasio.hpp
  src/messaging/sock/option.hpp
  src/messaging/sock/receive.hpp
  src/messaging/sock/resolve.hpp
  src/messaging/sock/send.hpp
  src/messaging/sock/traits.hpp
)

set(QIPERF_H
  qi/perf/dataperfsuite.hpp
  qi/perf/detail/dataperfsuite.hxx
  qi/perf/dataperf.hpp
  qi/perf/measure.hpp
)

set(QIPERF_C
  src/perf/dataperfsuite_p.hpp
  src/perf/dataperf_p.hpp
  src/perf/dataperfsuite.cpp
  src/perf/dataperf.cpp
  src/perf/measure.cpp
)

set(KA_H
  ka/algorithm.hpp
  ka/ark/mutable.hpp
  ka/concept.hpp
  ka/conceptpredicate.hpp
  ka/empty.hpp
  ka/errorhandling.hpp
  ka/functional.hpp
  ka/integersequence.hpp
  ka/macro.hpp
  ka/macroregular.hpp
  ka/memory.hpp
  ka/moveoncopy.hpp
  ka/mutablestore.hpp
  ka/mutex.hpp
  ka/opt.hpp
  ka/range.hpp
  ka/relationpredicate.hpp
  ka/scoped.hpp
  ka/sha1.hpp
  ka/src.hpp
  ka/testutils.hpp
  ka/typetraits.hpp
  ka/unit.hpp
  ka/utility.hpp
)

configure_file(qi/config.hpp.in ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/qi/config.hpp)
include_directories(${CATKIN_DEVEL_PREFIX}/include)

if(WITH_BOOST_LOCALE)
  list(APPEND QI_H qi/translator.hpp)
  list(APPEND QI_C src/translator.cpp)
endif()

install(DIRECTORY qi ka DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/qi/config.hpp DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/qi)

if (WIN32)
  list(APPEND QI_C
    src/os_launch_win32.cpp
    src/os_win32.cpp
    src/os_debugger_win32.cpp
  )
else()
  list(APPEND QI_C
    src/os_posix.cpp
    src/os_debugger_posix.cpp
  )
  if(ANDROID)
    list(APPEND QI_C src/os_launch_android.cpp)
  else()
    list(APPEND QI_C src/os_launch_posix.cpp)
  endif()
endif()

#normalize BUILD_SHARED_LIBS, default ON
if (NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

# Set qi_STATIC_BUILD  used in config.hpp.in
set(qi_STATIC_BUILD OFF)
if (NOT BUILD_SHARED_LIBS) #lol
  set(qi_STATIC_BUILD ON)
endif()

# Convert ON/OFF to SHARED/STATIC for create_lib argument
set(_qilibtype)
if (BUILD_SHARED_LIBS OR ANDROID)
  set(_qilibtype SHARED)
else()
  set(_qilibtype STATIC)
endif()


include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

include(qimodule-config.cmake)

add_library(qi SHARED
                 ${QI_H}     ${QI_C}
                 ${QITYPE_H} ${QITYPE_C}
                 ${QIM_H}    ${QIM_C}
                 ${QI_SOCKET_H}
                 ${KA_H}
            )
find_package(Boost QUIET COMPONENTS chrono filesystem program_options regex system thread random)
target_link_libraries(qi ${Boost_LIBRARIES} OpenSSL::Crypto OpenSSL::SSL)

if (WITH_QT5_CORE)
  qi_use_lib(qi QT5_CORE)
endif()
if(WITH_BOOST_LOCALE)
  qi_use_lib(qi BOOST_LOCALE)
endif()

if (UNIX)
  target_link_libraries(qi pthread)
endif()

if(ANDROID)
  target_link_libraries(qi log)
endif()

if(UNIX AND NOT APPLE AND NOT ANDROID)
  target_link_libraries(qi dl rt)
endif()

if(WIN32)
  qi_use_lib(qi WSOCK IPHLPAPI PSAPI WSA) #Really?
endif()


install(TARGETS qi DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
