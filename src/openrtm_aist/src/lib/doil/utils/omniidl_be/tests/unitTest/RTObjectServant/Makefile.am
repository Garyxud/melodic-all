# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for RTObjectServant unit test
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

noinst_PROGRAMS = RTObjectServantTests

RTObjectServantTests_SOURCES = ../TestRunner.cpp
RTObjectServantTests_SOURCES += RTObjectServantTests.cpp
RTObjectServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
RTObjectServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
RTObjectServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

RTObjectServantTests_LDFLAGS = -L$(libdir)

RTObjectServantTests_LDADD   = -lomniORB4
RTObjectServantTests_LDADD  += -lomnithread
RTObjectServantTests_LDADD  += -lomniDynamic4
RTObjectServantTests_LDADD  += -lcoil
#RTObjectServantTests_LDADD  += -lcppunit
RTObjectServantTests_LDADD  += $(target_srcdir)/RTObjectServant.o
RTObjectServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
RTObjectServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

RTObjectServantTests_LDADD  += $(target_srcdir)/ComponentActionServant.o
RTObjectServantTests_LDADD  += $(target_srcdir)/LightweightRTObjectServant.o
RTObjectServantTests_LDADD  += $(target_srcdir)/SDOSystemElementServant.o
RTObjectServantTests_LDADD  += $(target_srcdir)/SDOServant.o

# all
all: do-test

# do tests
do-test:
	./RTObjectServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

