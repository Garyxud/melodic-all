# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for IceManager unit test
# @date   $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

#
# $Log$
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS= -I.                  \
	-I$(includedir)           \
	-I$(top_builddir)         \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/coil/include


AM_LDFLAGS= -L.                   \
	-L$(top_builddir)         \
	-L$(top_builddir)/src/lib \
	-L$(top_builddir)/src/lib/coil/lib


SUFFIXES = .ice .cpp
.ice.cpp:
	slice2cpp $<

# if this test uses other lib libHogeSkel.a or libHogeSkel.so
#
#noinst_LTLIBRARIES = libHogeSkel.la

#IDL_SOURCES = hoge.idl
#libHogeSkel_la_SOURCES =            \
#	$(IDL_SOURCES:.idl=Skel.cpp) \
#	$(IDL_SOURCES:.idl=Stub.cpp) \
#	$(IDL_SOURCES:.idl=Skel.h)   \
#	$(IDL_SOURCES:.idl=Stub.h)
#libHogeSkel_la_LIBADD = $(LIBS)

noinst_PROGRAMS = IceManagerTests

IceManagerTests_SOURCES = ../TestRunner.cpp EchoSample.cpp IceManagerTests.cpp
IceManagerTests_LDFLAGS = -L$(libdir) \
	-L$(top_builddir)/src/lib/coil/lib \
	-L$(top_builddir)/src/lib/doil \
	-L$(top_builddir)/src/lib/doil/ice
IceManagerTests_LDADD   = -lIce -lIceUtil -lcoil -ldoil -ldoilice -lcppunit 

# all
all: do-test

# do tests
do-test:
	./IceManagerTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

