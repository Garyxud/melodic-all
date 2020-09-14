# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ORBManager unit test
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

AM_CPPFLAGS= -fPIC -I.                  \
	-I$(includedir)           \
	-I$(top_builddir)         \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/coil/include

AM_LDFLAGS= -E -L.                   \
	-L$(top_builddir)         \
	-L$(top_builddir)/src/lib \
	-L$(top_builddir)/src/lib/coil/lib

SUFFIXES = .idl Skel.cpp Stub.cpp Stub.o Skel.o .ice .cpp

.idlSkel.cpp:
	$(top_srcdir)/build/makewrapper.py $< src/lib/doil/tests/ORBManager
	$(IDLC) $(IDLFLAGS) $<

EchoIceSkel.h: EchoIceSkel.ice
	slice2cpp $<
EchoIceSkel.cpp: EchoIceSkel.ice
	slice2cpp $<

noinst_LTLIBRARIES = libEchoCORBA.la libEchoIce.la

libEchoCORBA_la_SOURCES = \
	EchoCORBASkel.h   \
	EchoCORBASkel.cpp \
	EchoCORBA.cpp

libEchoCORBA_la_LDFLAGS = \
	-rpath `pwd` \
	-L$(libdir) \
	-L$(top_builddir)/src/lib/coil/lib \
	-L$(top_builddir)/src/lib/doil \
	-L$(top_builddir)/src/lib/doil/corba \
libEchoCORBA_la_LIBADD = -lcoil -ldoil -ldoilcorba -lomniORB4 -lomnithread

libEchoIce_la_SOURCES = \
	EchoIceSkel.h   \
	EchoIceSkel.cpp \
	EchoIce.cpp

libEchoIce_la_LDFLAGS = \
	-rpath `pwd` \
	-L$(libdir) \
	-L$(top_builddir)/src/lib/coil/lib \
	-L$(top_builddir)/src/lib/doil/.libs \
	-L$(top_builddir)/src/lib/doil/ice/.libs
libEchoIce_la_LIBADD = -lcoil -ldoil -ldoilice -lIce -lIceUtil

noinst_PROGRAMS = ORBManagerTests

ORBManagerTests_SOURCES = ../TestRunner.cpp ORBManagerTests.cpp EchoImpl.cpp
ORBManagerTests_LDFLAGS = -L$(libdir) \
	-L$(top_builddir)/src/lib/coil/lib \
	-L$(top_builddir)/src/lib/doil \
	-L$(top_builddir)/src/lib/doil/corba \
	-L$(top_builddir)/src/lib/doil/ice
ORBManagerTests_LDADD   = -lcoil -ldoil -lcppunit -ldoilcorba -lomniORB4 -lomnithread -lEchoCORBA -ldoilice -lIce -lIceUtil -lEchoIce

# all
all: do-test

# do tests
do-test:
	./ORBManagerTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

