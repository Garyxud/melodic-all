## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for HelloRTWorld component
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

AM_CPPFLAGS=-I$(top_srcdir) -I$(top_srcdir)/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/rtm -L$(top_builddir)/rtm/idl

noinst_LTLIBRARIES = libHelloRTWorld.la
libHelloRTWorld_la_SOURCES = HelloRTWorld.cpp HelloRTWorld.h
libHelloRTWorld_la_LIBADD = $(top_builddir)/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
libHelloRTWorld_la_FLAGS = -L$(top_builddir)/rtm/.lib

noinst_PROGRAMS = HelloRTWorldComp
HelloRTWorldComp_SOURCES = HelloRTWorldComp.cpp 
HelloRTWorldComp_LDADD = HelloRTWorld.o \
	$(top_builddir)/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
HelloRTWorldComp_LDFLAGS = -L$(top_builddir)/rtm/.libs

EXTRA_DIST = run.sh

clean-local:
	rm -f *.bak *.rpo *.sym lib*.*_pure_* Makefile.old *core *~
