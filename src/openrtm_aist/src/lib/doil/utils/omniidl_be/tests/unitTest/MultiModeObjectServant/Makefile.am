# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for MultiModeObjectServant unit test
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

noinst_PROGRAMS = MultiModeObjectServantTests

MultiModeObjectServantTests_SOURCES = ../TestRunner.cpp
MultiModeObjectServantTests_SOURCES += MultiModeObjectServantTests.cpp
MultiModeObjectServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
MultiModeObjectServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
MultiModeObjectServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

MultiModeObjectServantTests_LDFLAGS = -L$(libdir)

MultiModeObjectServantTests_LDADD   = -lomniORB4
MultiModeObjectServantTests_LDADD  += -lomnithread
MultiModeObjectServantTests_LDADD  += -lomniDynamic4
MultiModeObjectServantTests_LDADD  += -lcoil
#MultiModeObjectServantTests_LDADD  += -lcppunit
MultiModeObjectServantTests_LDADD  += $(target_srcdir)/MultiModeObjectServant.o
MultiModeObjectServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
MultiModeObjectServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

MultiModeObjectServantTests_LDADD  += $(target_srcdir)/ComponentActionServant.o
MultiModeObjectServantTests_LDADD  += $(target_srcdir)/LightweightRTObjectServant.o
MultiModeObjectServantTests_LDADD  += $(target_srcdir)/MultiModeComponentActionServant.o
MultiModeObjectServantTests_LDADD  += $(target_srcdir)/ModeCapableServant.o

# all
all: do-test

# do tests
do-test:
	./MultiModeObjectServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

