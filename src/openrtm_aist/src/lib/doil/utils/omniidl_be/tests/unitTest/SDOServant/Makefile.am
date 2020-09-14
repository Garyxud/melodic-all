# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for SDOServant unit test
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

noinst_PROGRAMS = SDOServantTests

SDOServantTests_SOURCES = ../TestRunner.cpp
SDOServantTests_SOURCES += SDOServantTests.cpp
SDOServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
SDOServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
SDOServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

SDOServantTests_LDFLAGS = -L$(libdir)

SDOServantTests_LDADD   = -lomniORB4
SDOServantTests_LDADD  += -lomnithread
SDOServantTests_LDADD  += -lomniDynamic4
SDOServantTests_LDADD  += -lcoil
#SDOServantTests_LDADD  += -lcppunit
SDOServantTests_LDADD  += $(target_srcdir)/SDOServant.o
SDOServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
SDOServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

SDOServantTests_LDADD  += $(target_srcdir)/SDOSystemElementServant.o

# all
all: do-test

# do tests
do-test:
	./SDOServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

