# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for RTCUtil unit test
# @date   $Date: 2008/05/02 11:29:13 $
#
# $Id: Makefile.am,v 1.1 2008/05/02 11:29:13 arafune Exp $
#
#------------------------------------------------------------

#
# $Log: Makefile.am,v $
# Revision 1.1  2008/05/02 11:29:13  arafune
# The first commitment.
#
# Revision 1.1  2008/04/24 08:50:14  arafune
# The first commitment.
#
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS = \
	-I. \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/rtm \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(includedir)

AM_LDFLAGS = \
	-L. \
	-L$(top_builddir)

IDL_SOURCES = \
	$(top_srcdir)/src/lib/rtm/idl/SDOPackage.idl \
	$(top_srcdir)/src/lib/rtm/idl/RTC.idl \
	$(top_srcdir)/src/lib/rtm/idl/OpenRTM.idl
noinst_PROGRAMS = RTCUtilTests

nodist_RTCUtilTests_SOURCES = \
	../TestRunner.cpp \
	$(IDL_SOURCES:.idl=Skel.cpp) \
	$(top_srcdir)/src/lib/rtm/RTCUtil.cpp \
	RTCUtilTests.cpp

RTCUtilTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
RTCUtilTests_LDADD   = -lcppunit

EXTRA_DIST = \
	RTCUtilTests.cpp

# all
all: do-test

# do tests
do-test:
	./RTCUtilTests --xml rtcutiltests.xml \
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
