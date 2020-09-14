# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ExecutionContextServant unit test
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

noinst_PROGRAMS = ExecutionContextServantTests

ExecutionContextServantTests_SOURCES = ../TestRunner.cpp
ExecutionContextServantTests_SOURCES += ExecutionContextServantTests.cpp
ExecutionContextServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
ExecutionContextServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
ExecutionContextServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

ExecutionContextServantTests_LDFLAGS = -L$(libdir)

ExecutionContextServantTests_LDADD   = -lomniORB4
ExecutionContextServantTests_LDADD  += -lomnithread
ExecutionContextServantTests_LDADD  += -lomniDynamic4
ExecutionContextServantTests_LDADD  += -lcoil
#ExecutionContextServantTests_LDADD  += -lcppunit
ExecutionContextServantTests_LDADD  += $(target_srcdir)/ExecutionContextServant.o
ExecutionContextServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
ExecutionContextServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

# all
all: do-test

# do tests
do-test:
	./ExecutionContextServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

