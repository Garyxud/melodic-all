# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ManagerConfig unit test
# @date   $Date: 2007/01/21 06:52:12 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS =\
	-I. \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(top_builddir)/src/lib/rtm \
	-I$(includedir)

AM_LDFLAGS = \
	-L. \
	-L$(top_builddir) \
	-L$(top_builddir)/src/lib/coil/lib

noinst_PROGRAMS = ManagerConfigTests

nodist_ManagerConfigTests_SOURCES =\
	../TestRunner.cpp\
	$(top_builddir)/src/lib/rtm/ManagerConfig.cpp \
	ManagerConfigTests.cpp

ManagerConfigTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
ManagerConfigTests_LDADD   = -lcppunit -lcoil

EXTRA_DIST =               \
	ManagerConfigTests.cpp \
	rtc.conf

# all
all: do-test

# do tests
do-test:
	./ManagerConfigTests --xml managerconfigtests.xml \
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
