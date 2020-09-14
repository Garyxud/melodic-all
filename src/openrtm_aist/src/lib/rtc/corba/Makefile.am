## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for librtccorba.so
##
## $Id$
##---------------------------------------------------------------------------

SUBDIRS = idl .

AM_CPPFLAGS= \
	-I$(top_srcdir)/src/lib/coil/include \
	-I$(top_srcdir)/src/lib \
	-I$(top_srcdir)/src/lib/rtc/corba/idl

AM_LDFLAGS= \
	-L$(top_builddir)/src/lib/coil/lib \
	-L$(top_builddir)/src/lib/doil

lib_LTLIBRARIES = librtccorba.la
librtccorba_la_SOURCES =    \
	ExecutionContextAdapter.cpp

librtccorba_la_LIBADD = \
	$(top_builddir)/src/lib/rtc/corba/idl/libRTCSkel.la
librtccorba_la_LDFLAGS =            \
	-no-undefined               \
	-release $(PACKAGE_VERSION) \
	-version-info 0:4:0

