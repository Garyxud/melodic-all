# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for SDOSystemElementServant unit test
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

noinst_PROGRAMS = SDOSystemElementServantTests

SDOSystemElementServantTests_SOURCES = ../TestRunner.cpp
SDOSystemElementServantTests_SOURCES += SDOSystemElementServantTests.cpp
SDOSystemElementServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
SDOSystemElementServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
SDOSystemElementServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

SDOSystemElementServantTests_LDFLAGS = -L$(libdir)

SDOSystemElementServantTests_LDADD   = -lomniORB4
SDOSystemElementServantTests_LDADD  += -lomnithread
SDOSystemElementServantTests_LDADD  += -lomniDynamic4
SDOSystemElementServantTests_LDADD  += -lcoil
#SDOSystemElementServantTests_LDADD  += -lcppunit
SDOSystemElementServantTests_LDADD  += $(target_srcdir)/SDOSystemElementServant.o
SDOSystemElementServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
SDOSystemElementServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

# all
all: do-test

# do tests
do-test:
	./SDOSystemElementServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

