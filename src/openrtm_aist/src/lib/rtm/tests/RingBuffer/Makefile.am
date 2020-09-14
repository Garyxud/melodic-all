# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for RingBuffer unit test
# @date   $Date: 2007/01/21 06:53:26 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

#
# $Log: Makefile.am,v $
# Revision 1.3  2007/01/21 06:53:26  n-ando
# The include search path option order in AM_CPPFLAGS was changed.
#
# Revision 1.2  2007/01/12 14:45:19  n-ando
# Some fixes for distribution control.
#
# Revision 1.1  2006/11/27 08:36:59  n-ando
# TestSuites are devided into each directory.
#
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS =             \
	-I.               \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/rtm \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(includedir)


AM_LDFLAGS =              \
	-L.               \
	-L$(top_builddir) \
	-L$(top_builddir)/src/lib/coil/lib

noinst_PROGRAMS = RingBufferTests

nodist_RingBufferTests_SOURCES =     \
	../TestRunner.cpp            \
	RingBufferTests.cpp

RingBufferTests_LDFLAGS = -L../../ -L$(libdir) $(AM_LDFLAGS)
RingBufferTests_LDADD   = -lcppunit -lcoil -lRTC

EXTRA_DIST =            \
	RingBufferTests.cpp

# all
all: do-test

# do tests
do-test:
	./RingBufferTests --xml ringbuffertests.xml \
	--xsl default.xsl

# clean-up
clean-local:
	rm -f                               \
        *.log *.xml                         \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl
