## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for RT preempt linux kternel execution context
##
## $Id$
##---------------------------------------------------------------------------

AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

#------------------------------------------------------------
# Targets
#------------------------------------------------------------
# Targets directories
moduledir = $(rtm_ecdir)

module_LTLIBRARIES = RTPreemptEC.la

RTPreemptEC_la_SOURCES = RTPreemptEC.cpp RTPreemptEC.h
RTPreemptEC_la_LDFLAGS = -module -shared
RTPreemptEC_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.lib
RTPreemptEC_la_LIBADD  = \
	-lrt                                  \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la 

clean-local:
	rm -rf *~ *.o *.so
