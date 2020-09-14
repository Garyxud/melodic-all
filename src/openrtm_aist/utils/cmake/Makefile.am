## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile for cmake: creating OpenRTM-aist's FindOpenRTM.cmake for cmake
##
## $Id$
##---------------------------------------------------------------------------
#
# CMake's OpenRTMFind.cmake installation diretories
#
# * for the Windows systems
#  <prefix>/                                               (W)
#  <prefix>/(cmake|CMake)/                                 (W)
#  <prefix>/<name>*/                                       (W)
#  <prefix>/<name>*/(cmake|CMake)/                         (W)
#
# * for the UNIX systems
#  <prefix>/(share|lib)/cmake/<name>*/                     (U)
#  <prefix>/(share|lib)/<name>*/                           (U)
#  <prefix>/(share|lib)/<name>*/(cmake|CMake)/             (U)
#
# In Mac OS X with MacPorts, <prefix> might be /opt/local. (not confirmed)
# And on systems supporting OS X Frameworks and Application Bundles
# the following directories are searched for frameworks or bundles
# containing a configuration file:
#
# * for Mac OS X with frameworks and Application Bundles
#  <prefix>/<name>.framework/Resources/                    (A)
#  <prefix>/<name>.framework/Resources/CMake/              (A)
#  <prefix>/<name>.framework/Versions/*/Resources/         (A)
#  <prefix>/<name>.framework/Versions/*/Resources/CMake/   (A)
#  <prefix>/<name>.app/Contents/Resources/                 (A)
#  <prefix>/<name>.app/Contents/Resources/CMake/           (A)
#
# for the OpenRTM-aist
#
# Windows:
#   <prefix>/<name>*/cmake: C:/Progam Files/OpenRTM-aist/1.1/cmake
# UNIX:
#   <prefix>/lib/<name>*/cmake: /usr/lib/openrtm-1.1/cmake
# Mac OS X
#   <prefix>/lib/<name>*/cmake: /opt/local/lib/openrtm-1.1/cmake
#
# Users' CMakeList.txt just only use the following find_package() command
#   find_package(OpenRTM-aist ....)
#                ^^^^^^^^^^^^ cmake handle <name> case-insensitively
#
#
#
# testing find_package ability
# 
# Windows)
# configfile: C:\Program Files\OpenRTM-aist\1.1\cmake\
# directives:
#   find_package(OpenRTM CONFIGS OpenRTMConfig.cmake): OK
#   find_package(openrtm CONFIGS OpenRTMConfig.cmake): OK
#   find_package(OpenRTM-aist CONFIGS OpenRTMConfig.cmake): OK
#   find_package(openrtm-aist CONFIGS OpenRTMConfig.cmake): OK
#
# UNIX)
# configfile: /usr/lib/openrtm-1.1/cmake/OpenRTMConfig.cmake
# directives:
#   find_package(openrtm CONFIGS OpenRTMConfig.cmake): OK
#   find_package(OpenRTM CONFIGS OpenRTMConfig.cmake): OK
#   find_package(OpenRTM-aist CONFIGS OpenRTMConfig.cmake): NG
#   find_package(openrtm-aist CONFIGS OpenRTMConfig.cmake): NG
#   find_package(OpenRTM-aist CONFIGS OpenRTMConfig.cmake NAMES openrtm): OK
#   find_package(openrtm-aist CONFIGS OpenRTMConfig.cmake NAMES openrtm): OK
#

AUTOMAKE_OPTIONS = 1.4

ORB = @ORB@
IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@

cmakedir = ${rtm_libdir}/cmake
cmake_DATA = OpenRTMConfig.cmake FindOpenRTM.cmake

# basic compiler/linker options
OPENRTM_CFLAGS=`echo "${CXXFLAGS}" | sed -e 's/\-I[^ ]*//g'`
OPENRTM_INCLUDE_DIRS=        \
	-I${prefix}/include  \
	-I${coil_includedir} \
	-I${rtm_includedir}  \
	-I${rtm_includedir}/rtm/idl
OPENRTM_LDFLAGS_=`echo "${LDFLAGS}" | sed -e 's/\-L[^ ]*//g'`
OPENRTM_LIBRARY_DIRS=-L${prefix}/lib
OPENRTM_LIBRARIES_=${LIBS} -lRTC -lcoil

# OpenRTM-aist specific directory
COIL_INCLUDE_DIR=${coil_includedir}
OPENRTM_INCLUDE_DIR=${rtm_includedir}
OPENRTM_LIB_DIR=${rtm_libdir}
OPENRTM_DATA_DIR=${rtm_datadir}

# OpenRTM-aist version
OPENRTM_VERSION=${RTM_VERSION}
OPENRTM_VERSION_MAJOR=${RTM_MAJOR_VERSION}
OPENRTM_VERSION_MINOR=${RTM_MINOR_VERSION}
OPENRTM_REVISION_NUM=${RTM_REVISION_NUM}
OPENRTM_SHORT_VERSION=${RTM_SHORT_VERSION}

# OpenRTM-aist's CORBA related settings
OPENRTM_ORB=${ORB}
OPENRTM_IDL_WRAPPER=rtm-skelwrapper
OPENRTM_IDL_WRAPPER_FLAGS=--include-dir="" --skel-suffix=Skel --stub-suffix=Stub
OPENRTM_IDLC=${IDLC}
OPENRTM_IDLFLAGS=${IDLFLAGS}


OpenRTMConfig.cmake: Makefile.am
	echo "# -*- cmake -*-" > $@
	echo "#" >> $@
	echo "# @file OpenRTMConfig.cmake" >> $@
	echo "# @brief cmake-config file for OpenRTM-aist" >> $@
	echo "# @author Noriaki Ando" >> $@
	echo "# @date "`date` >> $@
	echo "#" >> $@
	echo "# This file is used for cmake config-mode." >> $@
	echo "# The following variables are defined." >> $@
	echo "#" >> $@
	echo "# Basic compiler/linker options" >> $@
	echo "# - OPENRTM_CFLAGS: cflags " >> $@
	echo "# - OPENRTM_INCLUDE_DIRS: include directories" >> $@
	echo "# - OPENRTM_LDFLAGS: linker options" >> $@
	echo "# - OPENRTM_LIBRARY_DIRS: library directories" >> $@
	echo "# - OPENRTM_LIBRARIES: libraries" >> $@
	echo "# - OPENRTM_LIB_DIR: OpenRTM's lib directory" >> $@
	echo "#" >> $@
	echo "# OpenRTM-aist specific directory" >> $@
	echo "# - COIL_INCLUDE_DIR: coil include dir" >> $@
	echo "# - OPENRTM_INCLUDE_DIR: OpenRTM's include directory" >> $@
	echo "# - OPENRTM_LIB_DIR: OpenRTM's lib directory" >> $@
	echo "# - OPENRTM_DATA_DIR: OpenRTM's shared directory" >> $@
	echo "#" >> $@
	echo "# OpenRTM-aist version" >> $@
	echo "# - OPENRTM_VERSION: x.y.z version" >> $@
	echo "# - OPENRTM_VERSION_MAJOR: major version number" >> $@
	echo "# - OPENRTM_VERSION_MINOR: minor version number" >> $@
	echo "# - OPENRTM_VERSION_PATCH: revision number" >> $@
	echo "# - OPENRTM_SHORT_VERSION: short version number 1.1.0->110" >> $@
	echo "#" >> $@
	echo "# OpenRTM-aist's CORBA related settings" >> $@
	echo "# - OPENRTM_ORB: CORBA implementation" >> $@
	echo "# - OPENRTM_IDL_WRAPPER: rtm-skelwrapper command" >> $@
	echo "# - OPENRTM_IDL_WRAPPER_FLAGS: rtm-skelwrapper flag" >> $@
	echo "# - OPENRTM_IDLC: IDL command" >> $@
	echo "# - OPENRTM_IDLFLAGS: IDL optins" >> $@
	echo "" >> $@
	echo 'message(STATUS "OpenRTMConfig.cmake found.")' >> $@
	echo 'message(STATUS "Configrued by configuration mode.")' >> $@
	echo "" >> $@
	echo "" >> $@
	echo "# Basic compiler/linker options" >> $@
	echo "set(OPENRTM_CFLAGS ${OPENRTM_CFLAGS})" >> $@
	echo "set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS})" >> $@
	echo "set(OPENRTM_LDFLAGS ${OPENRTM_LDFLAGS_})" >> $@
	echo "set(OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS})" >> $@
	echo "set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES_})" >> $@
	echo "set(OPENRTM_LIB_DIR ${OPENRTM_LIB_DIR})" >> $@
	echo "" >> $@
	echo "# OpenRTM-aist specific directory" >> $@
	echo "set(COIL_INCLUDE_DIR ${COIL_INCLUDE_DIR})" >> $@
	echo "set(OPENRTM_INCLUDE_DIR ${OPENRTM_INCLUDE_DIR})" >> $@
	echo "set(OPENRTM_LIB_DIR ${OPENRTM_LIB_DIR})" >> $@
	echo "set(OPENRTM_DATA_DIR ${OPENRTM_DATA_DIR})" >> $@
	echo "" >> $@
	echo "# OpenRTM-aist version" >> $@
	echo "set(OPENRTM_VERSION ${OPENRTM_VERSION})" >> $@
	echo "set(OPENRTM_VERSION_MAJOR ${OPENRTM_VERSION_MAJOR})" >> $@
	echo "set(OPENRTM_VERSION_MINOR ${OPENRTM_VERSION_MINOR})" >> $@
	echo "set(OPENRTM_VERSION_PATCH ${OPENRTM_REVISION_NUM})" >> $@
	echo "set(OPENRTM_SHORT_VERSION ${OPENRTM_SHORT_VERSION})" >> $@
	echo "" >> $@
	echo "# OpenRTM-aist's CORBA related settings" >> $@
	echo "set(OPENRTM_ORB ${OPENRTM_ORB})" >> $@
	echo "set(OPENRTM_IDL_WRAPPER ${OPENRTM_IDL_WRAPPER})" >> $@
	echo "set(OPENRTM_IDL_WRAPPER_FLAGS ${OPENRTM_IDL_WRAPPER_FLAGS})" >> $@
	echo "set(OPENRTM_IDLC ${OPENRTM_IDLC})" >> $@
	echo "set(OPENRTM_IDLFLAGS ${OPENRTM_IDLFLAGS})" >> $@
	echo "" >> $@
	echo 'message(STATUS "OpenRTM-aist configuration done")' >> $@
	echo "# end of OpenRTMConfig.cmake" >> $@


EXTRA_DIST = FindOpenRTM.cmake

clean-local:
	rm -f OpenRTMConfig.cmake
	rm -rf *~
