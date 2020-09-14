# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for FsmServant unit test
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

noinst_PROGRAMS = FsmServantTests

FsmServantTests_SOURCES = ../TestRunner.cpp
FsmServantTests_SOURCES += FsmServantTests.cpp
FsmServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
FsmServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
FsmServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

FsmServantTests_LDFLAGS = -L$(libdir)

FsmServantTests_LDADD   = -lomniORB4
FsmServantTests_LDADD  += -lomnithread
FsmServantTests_LDADD  += -lomniDynamic4
FsmServantTests_LDADD  += -lcoil
#FsmServantTests_LDADD  += -lcppunit
FsmServantTests_LDADD  += $(target_srcdir)/FsmServant.o
FsmServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
FsmServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

FsmServantTests_LDADD  += $(target_srcdir)/ComponentActionServant.o
FsmServantTests_LDADD  += $(target_srcdir)/LightweightRTObjectServant.o

# all
all: do-test

# do tests
do-test:
	./FsmServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

