# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for FsmParticipantServant unit test
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

noinst_PROGRAMS = FsmParticipantServantTests

FsmParticipantServantTests_SOURCES = ../TestRunner.cpp
FsmParticipantServantTests_SOURCES += FsmParticipantServantTests.cpp
FsmParticipantServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
FsmParticipantServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
FsmParticipantServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

FsmParticipantServantTests_LDFLAGS = -L$(libdir)

FsmParticipantServantTests_LDADD   = -lomniORB4
FsmParticipantServantTests_LDADD  += -lomnithread
FsmParticipantServantTests_LDADD  += -lomniDynamic4
FsmParticipantServantTests_LDADD  += -lcoil
#FsmParticipantServantTests_LDADD  += -lcppunit
FsmParticipantServantTests_LDADD  += $(target_srcdir)/FsmParticipantServant.o
FsmParticipantServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
FsmParticipantServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

FsmParticipantServantTests_LDADD  += $(target_srcdir)/FsmParticipantActionServant.o
FsmParticipantServantTests_LDADD  += $(target_srcdir)/ComponentActionServant.o
FsmParticipantServantTests_LDADD  += $(target_srcdir)/LightweightRTObjectServant.o

# all
all: do-test

# do tests
do-test:
	./FsmParticipantServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

