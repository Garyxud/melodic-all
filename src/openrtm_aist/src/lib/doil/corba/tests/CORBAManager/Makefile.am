# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for CORBAManager unit test
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


SUFFIXES = .idl Skel.cpp Stub.cpp Stub.o Skel.o
.idlSkel.cpp:
	$(top_srcdir)/build/makewrapper.py $< src/lib/doil/corba/tests/CORBAManager
	$(IDLC) $(IDLFLAGS) $<

# if this test uses other lib libHogeSkel.a or libHogeSkel.so
#
noinst_LTLIBRARIES = libEchoSkel.la

IDL_SOURCES = Echo.idl
libEchoSkel_la_SOURCES =            \
	$(IDL_SOURCES:.idl=Skel.cpp) \
	$(IDL_SOURCES:.idl=Skel.h)

libEchoSkel_la_LIBADD = $(LIBS)

noinst_PROGRAMS = CORBAManagerTests

CORBAManagerTests_SOURCES = ../TestRunner.cpp CORBAManagerTests.cpp
CORBAManagerTests_LDFLAGS = -L$(libdir) \
	-L$(top_builddir)/src/lib/coil/lib \
	-L$(top_builddir)/src/lib/doil \
	-L$(top_builddir)/src/lib/doil/corba
CORBAManagerTests_LDADD   =  libEchoSkel.la -lcoil -ldoil -ldoilcorba -lcppunit

# all
all: do-test

# do tests
do-test:
	./CORBAManagerTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

