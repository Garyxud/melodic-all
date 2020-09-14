# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for FsmServiceServant unit test
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

noinst_PROGRAMS = FsmServiceServantTests

FsmServiceServantTests_SOURCES = ../TestRunner.cpp
FsmServiceServantTests_SOURCES += FsmServiceServantTests.cpp
FsmServiceServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
FsmServiceServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
FsmServiceServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

FsmServiceServantTests_LDFLAGS = -L$(libdir)

FsmServiceServantTests_LDADD   = -lomniORB4
FsmServiceServantTests_LDADD  += -lomnithread
FsmServiceServantTests_LDADD  += -lomniDynamic4
FsmServiceServantTests_LDADD  += -lcoil
#FsmServiceServantTests_LDADD  += -lcppunit
FsmServiceServantTests_LDADD  += $(target_srcdir)/FsmServiceServant.o
FsmServiceServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
FsmServiceServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

FsmServiceServantTests_LDADD  += $(target_srcdir)/SDOServiceServant.o

# all
all: do-test

# do tests
do-test:
	./FsmServiceServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

