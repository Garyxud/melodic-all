# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for PortServiceServant unit test
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

noinst_PROGRAMS = PortServiceServantTests

PortServiceServantTests_SOURCES = ../TestRunner.cpp
PortServiceServantTests_SOURCES += PortServiceServantTests.cpp
PortServiceServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
PortServiceServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
PortServiceServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

PortServiceServantTests_LDFLAGS = -L$(libdir)

PortServiceServantTests_LDADD   = -lomniORB4
PortServiceServantTests_LDADD  += -lomnithread
PortServiceServantTests_LDADD  += -lomniDynamic4
PortServiceServantTests_LDADD  += -lcoil
#PortServiceServantTests_LDADD  += -lcppunit
PortServiceServantTests_LDADD  += $(target_srcdir)/PortServiceServant.o
PortServiceServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
PortServiceServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

PortServiceServantTests_LDADD  += $(target_srcdir)/SDOServiceServant.o

# all
all: do-test

# do tests
do-test:
	./PortServiceServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

