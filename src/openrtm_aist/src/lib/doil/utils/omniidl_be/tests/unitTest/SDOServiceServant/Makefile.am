# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for SDOServiceServant unit test
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

open_rtm_dir = $(top_builddir)/../../../../..
target_srcdir = $(top_builddir)/..

AM_CPPFLAGS = -I.
AM_CPPFLAGS += -I$(includedir)
AM_CPPFLAGS += -I$(target_srcdir)
AM_CPPFLAGS += -I$(open_rtm_dir)
AM_CPPFLAGS += -I$(open_rtm_dir)/rtc/corba/idl

AM_LDFLAGS = -L.
AM_LDFLAGS += -L$(open_rtm_dir)/coil/lib

IDL_SOURCES = $(open_rtm_dir)/rtc/corba/idl/SDOPackage.idl
IDL_SOURCES += $(open_rtm_dir)/rtc/corba/idl/RTC.idl
IDL_SOURCES += $(open_rtm_dir)/rtc/corba/idl/OpenRTM.idl

noinst_PROGRAMS = SDOServiceServantTests

SDOServiceServantTests_SOURCES = ../TestRunner.cpp
SDOServiceServantTests_SOURCES += SDOServiceServantTests.cpp
SDOServiceServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
SDOServiceServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
SDOServiceServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

SDOServiceServantTests_LDFLAGS = -L$(libdir)

SDOServiceServantTests_LDADD   = -lomniORB4
SDOServiceServantTests_LDADD  += -lomnithread
SDOServiceServantTests_LDADD  += -lomniDynamic4
SDOServiceServantTests_LDADD  += -lcoil
#SDOServiceServantTests_LDADD  += -lcppunit
SDOServiceServantTests_LDADD  += $(target_srcdir)/SDOServiceServant.o
SDOServiceServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
SDOServiceServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

# all
all: do-test

# do tests
do-test:
	./SDOServiceServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

