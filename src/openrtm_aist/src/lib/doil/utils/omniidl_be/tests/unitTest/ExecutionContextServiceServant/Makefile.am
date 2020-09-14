# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ExecutionContextServiceServant unit test
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

noinst_PROGRAMS = ExecutionContextServiceServantTests

ExecutionContextServiceServantTests_SOURCES = ../TestRunner.cpp
ExecutionContextServiceServantTests_SOURCES += ExecutionContextServiceServantTests.cpp
ExecutionContextServiceServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
ExecutionContextServiceServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
ExecutionContextServiceServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

ExecutionContextServiceServantTests_LDFLAGS = -L$(libdir)

ExecutionContextServiceServantTests_LDADD   = -lomniORB4
ExecutionContextServiceServantTests_LDADD  += -lomnithread
ExecutionContextServiceServantTests_LDADD  += -lomniDynamic4
ExecutionContextServiceServantTests_LDADD  += -lcoil
#ExecutionContextServiceServantTests_LDADD  += -lcppunit
ExecutionContextServiceServantTests_LDADD  += $(target_srcdir)/ExecutionContextServiceServant.o
ExecutionContextServiceServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
ExecutionContextServiceServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

ExecutionContextServiceServantTests_LDADD  += $(target_srcdir)/SDOServiceServant.o
ExecutionContextServiceServantTests_LDADD  += $(target_srcdir)/ExecutionContextServant.o

# all
all: do-test

# do tests
do-test:
	./ExecutionContextServiceServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

