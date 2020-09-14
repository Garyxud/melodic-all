# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for SdoService unit test
# @date   $Date: 2007/01/21 06:53:43 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS=              \
	-I.               \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib/rtm \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(includedir)


AM_LDFLAGS=               \
	-L.               \
	-L$(top_builddir)


noinst_PROGRAMS = SdoServiceTests

nodist_SdoServiceTests_SOURCES =     \
	../TestRunner.cpp            \
	SdoServiceTests.cpp

SdoServiceTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
SdoServiceTests_LDADD   = -lcppunit

EXTRA_DIST =            \
	SdoServiceTests.cpp

# all
all: do-test

# do tests
do-test:
	./SdoServiceTests --xml sdoservicetests.xml \
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

