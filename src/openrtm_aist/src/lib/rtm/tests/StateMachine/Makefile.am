# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for StateMachine unit test
# @date   $Date: 2007/01/21 06:53:49 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

#
# $Log: Makefile.am,v $
# Revision 1.3  2007/01/21 06:53:49  n-ando
# The include search path option order in AM_CPPFLAGS was changed.
#
# Revision 1.2  2007/01/12 14:45:43  n-ando
# Some fixes for distribution control.
#
# Revision 1.1  2006/11/27 08:30:04  n-ando
# TestSuites are devided into each directory.
#
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS= \
	-I. \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib/rtm \
	-I$(top_builddir)/src/lib/ \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(includedir)

AM_LDFLAGS=               \
	-L.               \
	-L$(top_builddir)


noinst_PROGRAMS = StateMachineTests

nodist_StateMachineTests_SOURCES =     \
	../TestRunner.cpp              \
	StateMachineTests.cpp

StateMachineTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
StateMachineTests_LDADD   = -lcppunit

EXTRA_DIST = \
	StateMachineTests.cpp

# all
all: do-test

# do tests
do-test:
	./StateMachineTests --xml statemachinetests.xml \
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
