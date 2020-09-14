## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for StringIn/StringOut components
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

AM_CPPFLAGS=-I$(top_srcdir) -I$(top_srcdir)/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/rtm -L$(top_builddir)/rtm/idl

noinst_LTLIBRARIES = libStringIn.la libStringOut.la
libStringIn_la_SOURCES = StringIn.cpp StringIn.h
libStringIn_la_LIBADD = $(top_builddir)/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
libStringIn_la_FLAGS = -L$(top_builddir)/rtm/.lib

libStringOut_la_SOURCES = StringOut.cpp StringOut.h
libStringOut_la_LIBADD = $(top_builddir)/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
libStringOut_la_FLAGS = -L$(top_builddir)/rtm/.lib

noinst_PROGRAMS = StringInComp StringOutComp
StringInComp_SOURCES = StringInComp.cpp 
StringInComp_LDADD =  StringIn.o \
	$(top_builddir)/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
StringInComp_LDFLAGS = -L$(top_builddir)/rtm/.libs

StringOutComp_SOURCES = StringOutComp.cpp 
StringOutComp_LDADD =  StringOut.o \
	$(top_builddir)/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
StringOutComp_LDFLAGS = -L$(top_builddir)/rtm/.libs

EXTRA_DIST = \
	Makefile.StringIn \
	Makefile.StringOut

clean-local:
	rm -f *.bak *.rpo *.sym lib*.*_pure_* Makefile.old *core *~
