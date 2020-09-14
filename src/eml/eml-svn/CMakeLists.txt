# Copyright FMTC 2006-2008
# Author: Klaas Gadeyne.  Copied stuff from <http://cmake.org/HTML/Examples.html>
# Linux (PREEMPT_RT) port based on patch by RvdM (October, 5th, 2007)

# The name of our project is "EML" (EtherCat Master Library).
# CMakeLists files in this project can refer to the root source
# directory of the project as ${EML_SOURCE_DIR} and
# to the root binary directory of the project as ${EML_BINARY_DIR}.
project (EML)

OPTION( BUILD_FOR_ECOS "Build EML for the eCos OS" OFF)

IF (BUILD_FOR_ECOS)
   INCLUDE(UseEcos)
   SET( ECOS_INSTALL_PATH "/path/to/ecos.ecc file" CACHE STRING "Where are your ecos installation directories (i.e. your ecos.ecc file)?" )
   # ECOS_ADD_INCLUDE_DIRECTORIES()
   INCLUDE_DIRECTORIES(${EML_SOURCE_DIR}/include/ethercat/arch-eCos)
   INCLUDE_DIRECTORIES(${ECOS_INSTALL_PATH}/ecos/install/include/)
   # FIXME: Build system only supports i386 for now...
   ECOS_USE_I386_ELF_TOOLS()
ENDIF (BUILD_FOR_ECOS)

OPTION( BUILD_FOR_RTNET "Build EML using RTNET/Xenomai" OFF)

IF (BUILD_FOR_RTNET)
	SET(CMAKE_VERBOSE_MAKEFILE ON)
	SET(RTNET_INSTALL_PATH "/usr/local/rtnet" CACHE STRING "Path of the RTnet installation directory")
	SET(XENOMAI_INSTALL_PATH "/usr/xenomai" CACHE STRING "Path of the Xenomai installation directory")

	INCLUDE_DIRECTORIES(${EML_SOURCE_DIR}/include/ethercat/arch-RTnet)
	INCLUDE_DIRECTORIES(${RTNET_INSTALL_PATH}/include)
	INCLUDE_DIRECTORIES(${XENOMAI_INSTALL_PATH}/include)
	LINK_DIRECTORIES(${XENOMAI_INSTALL_PATH}/lib)

	SET (XNPOSIX_USER_CFLAGS "`${XENOMAI_INSTALL_PATH}/bin/xeno-config --posix-cflags`")
	EXEC_PROGRAM ("${XENOMAI_INSTALL_PATH}/bin/xeno-config --posix-ldflags"
	  OUTPUT_VARIABLE XNPOSIX_USER_LDFLAGS)
ENDIF (BUILD_FOR_RTNET)

OPTION( BUILD_FOR_POSIX "Build for POSIX (Only tested on Linux/PREEMPT_RT for now)" ON)

IF (BUILD_FOR_POSIX)
        SET (POSIX_USER_LDFLAGS "-lpthread -lrt")

	INCLUDE_DIRECTORIES(${EML_SOURCE_DIR}/include/ethercat/arch-RTnet)

ENDIF (BUILD_FOR_POSIX)

# Make sure the compiler can find include files from our library.
include_directories (${EML_SOURCE_DIR}/include)

# This simply branches into the listed subdirectories Recurse into the
# "src" and "test" subdirectories.  This does not actually cause
# another cmake executable to run.  The same process will walk through
# the project's entire directory structure.
add_subdirectory (src)
add_subdirectory (tests)

# FIXME KG What's the f*cking difference with SUBDIRS, as used in the
# ecos examples?
# SUBDIRS(src tests)


