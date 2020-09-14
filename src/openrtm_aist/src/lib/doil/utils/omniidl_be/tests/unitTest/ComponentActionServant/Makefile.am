# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ComponentActionServant unit test
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

noinst_PROGRAMS = ComponentActionServantTests

ComponentActionServantTests_SOURCES = ../TestRunner.cpp
ComponentActionServantTests_SOURCES += ComponentActionServantTests.cpp
ComponentActionServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
ComponentActionServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
ComponentActionServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

ComponentActionServantTests_LDFLAGS = -L$(libdir)

ComponentActionServantTests_LDADD   = -lomniORB4
ComponentActionServantTests_LDADD  += -lomnithread
ComponentActionServantTests_LDADD  += -lomniDynamic4
ComponentActionServantTests_LDADD  += -lcoil
#ComponentActionServantTests_LDADD  += -lcppunit
ComponentActionServantTests_LDADD  += $(target_srcdir)/ComponentActionServant.o
ComponentActionServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
ComponentActionServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

# all
all: do-test

# do tests
do-test:
	./ComponentActionServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

