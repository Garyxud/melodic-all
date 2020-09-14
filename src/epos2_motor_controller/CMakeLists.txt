# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
   
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(libepos2)
SET(MAJOR_VERSION 0)
SET(MINOR_VERSION 4)
SET(VERSION_STRING ${MAJOR_VERSION}.${MINOR_VERSION})
SET(VERSION ${VERSION_STRING})

# automatically set lib suffix
IF(UNIX AND NOT APPLE AND NOT CMAKE_CROSSCOMPILING AND NOT EXISTS "/etc/debian_version")
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT LIB_SUFFIX)
    SET(LIB_SUFFIX 64)
  ENDIF()
ENDIF()

# config script install path
IF(NOT DEFINED LIBEPOS2_CMAKE_CONFIG_DIR)
  SET( LIBEPOS2_CMAKE_CONFIG_DIR lib${LIB_SUFFIX}/cmake/libepos2)
ENDIF()


SET(prefix      ${CMAKE_INSTALL_PREFIX})
SET(exec_prefix ${CMAKE_INSTALL_PREFIX}/bin)
SET(includedir  ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})

IF(${UNIX})
   SET(libdir      ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX})
ENDIF(${UNIX})
IF(${WIN32})
   SET(libdir      ${CMAKE_INSTALL_PREFIX}/bin)
ENDIF(${WIN32})

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF (NOT CMAKE_BUILD_TYPE)

ADD_SUBDIRECTORY(src)

#FIND_PACKAGE(Doxygen)

#FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
#IF (IRI_DOC_DIR)
#  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
#ELSE (IRI_DOC_DIR)
#  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
#ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

install(FILES package.xml DESTINATION share/libepos2)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/CMakeFiles*
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/src*
    COMMAND rm ARGS CMakeCache.txt
    COMMAND rm ARGS cmake_install.cmake
    COMMAND rm ARGS install_manifest.txt
    COMMAND rm ARGS Makefile

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (dep @echo install package dependencies)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "make package dependencies"
    COMMAND ./make_dep.sh ARGS -d ../dependencies
    TARGET  dep
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "make package dependencies only implemented in unix"
    TARGET  dep
  )
ENDIF(UNIX)

IF (UNIX)
 SET(CPACK_PACKAGE_FILE_NAME "iri-${PROJECT_NAME}-dev-${CPACK_PACKAGE_VERSION}${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
 SET(CPACK_PACKAGE_NAME "iri-${PROJECT_NAME}-dev")
 SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "This is a driver for Maxon EPOS2 Motor Controller for linux EPOS2 part of IRI-laboratory libraries. More information at http://wikiri.upc.es/index.php/Robotics_Lab")
 SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
 SET(CPACK_GENERATOR "DEB")
 SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "galenya - labrobotica@iri.upc.edu")
 SET(CPACK_SET_DESTDIR "ON")  # Necessary because of the absolute install paths

 INCLUDE(CPack)
ELSE(UNIX)
 ADD_CUSTOM_COMMAND(
   COMMENT "packaging only implemented in unix"
   TARGET  uninstall
)
ENDIF(UNIX)
