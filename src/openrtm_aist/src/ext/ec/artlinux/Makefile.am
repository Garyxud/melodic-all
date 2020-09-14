## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for ART-Linux execution context
##
## $Id: Makefile.am 1429 2009-07-06 08:28:25Z hakuta $
##---------------------------------------------------------------------------

ARTLIB   = /usr/lib/art_syscalls.o

AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

#------------------------------------------------------------
# Targets
#------------------------------------------------------------
# Targets directories
moduledir = $(rtm_ecdir)

module_LTLIBRARIES = ArtExecutionContext.la

ArtExecutionContext_la_SOURCES = ArtExecutionContext.cpp ArtExecutionContext.h
ArtExecutionContext_la_LDFLAGS = -module -shared $(ARTLIB)
ArtExecutionContext_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.lib
ArtExecutionContext_la_LIBADD  = $(top_builddir)/src/lib/rtm/libRTC.la

clean:
	rm -rf *~ *.o *.so