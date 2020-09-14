# unset version variables
unset(PROJECT_VERSION)
unset(PACKAGE_VERSION)

if (MSVC)
  # Switching off compiler warnings in sub-modules and generated code
  add_definitions("/wd4013")
  add_definitions("/wd4061")
  add_definitions("/wd4062")
  add_definitions("/wd4090")
  add_definitions("/wd4100")
  add_definitions("/wd4125")
  add_definitions("/wd4127")
  add_definitions("/wd4242")
  add_definitions("/wd4255")
  add_definitions("/wd4265")
  add_definitions("/wd4267")
  add_definitions("/wd4296")
  add_definitions("/wd4310")
  add_definitions("/wd4312")
  add_definitions("/wd4365")
  add_definitions("/wd4388")
  add_definitions("/wd4389")
  add_definitions("/wd4456")
  add_definitions("/wd4457")
  add_definitions("/wd4505")
  add_definitions("/wd4514")
  add_definitions("/wd4548")
  add_definitions("/wd4574")
  add_definitions("/wd4623")
  add_definitions("/wd4625")
  add_definitions("/wd4626")
  add_definitions("/wd4647")
  add_definitions("/wd4668")
  add_definitions("/wd4701")
  add_definitions("/wd4702")
  add_definitions("/wd4703")
  add_definitions("/wd4710")
  add_definitions("/wd4711")
  add_definitions("/wd4738")
  add_definitions("/wd4774")
  add_definitions("/wd4820")
  add_definitions("/wd4946")
  add_definitions("/wd5026")
  add_definitions("/wd5027")
endif ()

# Submodule: CPR

if (WIN32)
  option(CMAKE_USE_OPENSSL "Use OpenSSL code. Experimental" OFF)
endif ()

## disabling tests for cpr as googletests cause some issue in catkin workflow:
#  "CMake Error at cpr/opt/googletest/cmake/internal_utils.cmake:145 (add_library):
#  add_library cannot create target "gtest" because another target with the
#  same name already exists.  The existing target is a shared library created
#  in source directory "/usr/src/gtest".  See documentation for policy CMP0002
#  for more details.
#  Call Stack (most recent call first):
#  cpr/opt/googletest/cmake/internal_utils.cmake:168 (cxx_library_with_type)
#  cpr/opt/googletest/CMakeLists.txt:80 (cxx_library)"
set(BUILD_CPR_TESTS OFF CACHE BOOL "Don't build cpr tests")

# build cpr as static library with position independent code
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (WIN32)
  set(USE_SYSTEM_CURL_DEFAULT OFF)
else ()
  set(USE_SYSTEM_CURL_DEFAULT ON)
endif ()

option(USE_SYSTEM_CURL "If ON, this project will look in the system paths for an installed curl library" ${USE_SYSTEM_CURL_DEFAULT})

if (USE_SYSTEM_CURL)
  find_package(CURL)
endif ()

if (NOT USE_SYSTEM_CURL OR NOT CURL_FOUND)
  set(USE_SYSTEM_CURL OFF CACHE BOOL "If ON, this project will look in the system paths for an installed curl library" FORCE)

  set(BUILD_CURL_EXE ON CACHE BOOL "Build CURL command line tool")
  set(BUILD_TESTING OFF CACHE BOOL "Build testing tree")
endif ()

add_subdirectory(cpr)

# Submodule: JSON

set(JSON_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/json/src PARENT_SCOPE)

# Submodule: Protobuf

option(USE_SYSTEM_PROTOBUF "If ON, this project will look in the system paths for an installed protobuf compiler and library" ON)

if (USE_SYSTEM_PROTOBUF)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules/")
  find_package(ProtocolBuffers)

  if (NOT PROTOBUF_LIBRARY OR NOT PROTOBUF_INCLUDE_DIR OR NOT PROTOBUF_PROTOC_EXECUTABLE)
    set(USE_SYSTEM_PROTOBUF OFF CACHE BOOL "If ON, this project will look in the system paths for an installed protobuf compiler or library" FORCE)
  endif ()
endif ()

if (NOT USE_SYSTEM_PROTOBUF)
  message(STATUS "Not using system Protobuf, using built-in protobuf project instead.")

  option(protobuf_BUILD_EXAMPLES "Build examples" OFF)
  option(protobuf_BUILD_SHARED_LIBS "Build shared libraries" OFF)
  option(protobuf_BUILD_TESTS "Build tests" OFF)
  option(protobuf_INSTALL_EXAMPLES "Build tests" OFF)

  if (WIN32)
    option(protobuf_MSVC_STATIC_RUNTIME "Set using static runtime under windows" OFF)
  endif ()

  add_subdirectory(protobuf/cmake)

  set(PROTOBUF_FOUND TRUE CACHE INTERNAL "Protobuf found")
  set(PROTOBUF_PROTOC_EXECUTABLE protoc CACHE INTERNAL "Protobuf protoc")
  set(PROTOBUF_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protobuf/src" CACHE INTERNAL "Protobuf include directory")
  set(PROTOBUF_LIBRARY libprotobuf CACHE INTERNAL "Protobuf libprotobuf")
  set(PROTOBUF_LIBRARIES "" CACHE INTERNAL "Protobuf support libraries")

  set_property(TARGET protoc PROPERTY FOLDER "external")
  set_property(TARGET libprotobuf PROPERTY FOLDER "external")
  set_property(TARGET libprotobuf-lite PROPERTY FOLDER "external")
  set_property(TARGET libprotoc PROPERTY FOLDER "external")
endif()
