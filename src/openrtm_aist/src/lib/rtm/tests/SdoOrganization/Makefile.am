# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for SdoOrganization unit test
# @date   $Date: 2007/01/21 06:53:36 $
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
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/rtm \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(includedir)


AM_LDFLAGS=               \
	-L.               \
	-L$(top_builddir)/src/lib/rtm/.libs \
	-L$(top_builddir)/src/lib/coil/lib


noinst_PROGRAMS = SdoOrganizationTests

nodist_SdoOrganizationTests_SOURCES =              \
	../TestRunner.cpp                          \
	SdoOrganizationTests.cpp


#	(IDL_SOURCES:.idl=Skel.cpp)               \
#	(top_builddir)/src/lib/rtm/SdoOrganization.cpp    

SdoOrganizationTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
SdoOrganizationTests_LDADD   = -lcppunit -lRTC -lcoil -luuid

EXTRA_DIST =                 \
	SdoOrganizationTests.cpp

# all
all: do-test

# do tests
do-test:
	./SdoOrganizationTests --xml sdoorganizationtests.xml \
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
