## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for libdoil
##
## $Id$
##------------------------------------------------------------


AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include
AM_LDFLAGS=-L$(top_builddir)/src/lib/doil -L$(top_builddir)/src/lib/coil/lib

SUBDIRS = . tests
DOILCORBA_SRC =               \
	CORBAManager.cpp


DOILCORBA_H =               \
	CORBAManager.h

DOILCORBA_ALL = $(DOILCORBA_SRC) $(DOILCORBA_H)

lib_LTLIBRARIES = libdoilcorba.la
libdoilcorba_la_SOURCES = \
	$(DOILCORBA_SRC)
libdoilcorba_la_LIBADD = -ldoil -lcoil $(LIBS)

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
#lst:
#	echo $(ALL_SRC) > skelstub.lst
#	echo $(IDL_SOURCES) > idls.lst

#------------------------------------------------------------
# Visual Studio Project
#------------------------------------------------------------
#win32_builddir=$(top_builddir)/win32/OpenRTM-aist/
#
#vcproj: vc8proj vc9proj
#
#vc8proj: libRTCSkel.vcproj.yaml
#	$(top_builddir)/build/vcprojtool.py vcproj \
#		--type LIB \
#		--vcversion "8.00" \
#		--version $(RTM_VERSION) \
#		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc8.vcproj \
#		--yaml libRTCSkel.vcproj.yaml \
#		--source $(SKEL_CPP) \
#		--header $(SKEL_H)
#	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc8.vcproj
#
#vc9proj: libRTCSkel.vcproj.yaml
#	$(top_builddir)/build/vcprojtool.py vcproj \
#		--type LIB \
#		--vcversion "9.00" \
#		--version $(RTM_VERSION) \
#		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc9.vcproj \
#		--yaml libRTCSkel.vcproj.yaml \
#		--source $(SKEL_CPP) \
#		--header $(SKEL_H)
#	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc9.vcproj
#
##------------------------------------------------------------
## for wxs file
##------------------------------------------------------------
#wxs:
#	$(top_srcdir)/build/makewxs.py flist \
#		-c IDLs \
#		-o $(win32_builddir)installer/idls.yaml \
#		-p ..\\rtm\\idl \
#		$(IDL_SOURCES) $(ALL_SRC)
#
#dist-hook: lst vcproj wxs

# Cleanup
clean-local:
	rm -f $(ALL_SRC) *~ *.core *.lst Makefile.old

# Extra sources included distribution
#EXTRA_DIST =




