# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for MultiModeComponentActionServant unit test
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

noinst_PROGRAMS = MultiModeComponentActionServantTests

MultiModeComponentActionServantTests_SOURCES = ../TestRunner.cpp
MultiModeComponentActionServantTests_SOURCES += MultiModeComponentActionServantTests.cpp
MultiModeComponentActionServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
MultiModeComponentActionServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
MultiModeComponentActionServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

MultiModeComponentActionServantTests_LDFLAGS = -L$(libdir)

MultiModeComponentActionServantTests_LDADD   = -lomniORB4
MultiModeComponentActionServantTests_LDADD  += -lomnithread
MultiModeComponentActionServantTests_LDADD  += -lomniDynamic4
MultiModeComponentActionServantTests_LDADD  += -lcoil
#MultiModeComponentActionServantTests_LDADD  += -lcppunit
MultiModeComponentActionServantTests_LDADD  += $(target_srcdir)/MultiModeComponentActionServant.o
MultiModeComponentActionServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
MultiModeComponentActionServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

# all
all: do-test

# do tests
do-test:
	./MultiModeComponentActionServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

