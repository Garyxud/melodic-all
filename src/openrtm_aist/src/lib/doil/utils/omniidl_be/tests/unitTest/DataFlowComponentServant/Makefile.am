# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for DataFlowComponentServant unit test
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

noinst_PROGRAMS = DataFlowComponentServantTests

DataFlowComponentServantTests_SOURCES = ../TestRunner.cpp
DataFlowComponentServantTests_SOURCES += DataFlowComponentServantTests.cpp
DataFlowComponentServantTests_SOURCES += $(IDL_SOURCES:.idl=Stub.cpp)
DataFlowComponentServantTests_SOURCES += $(open_rtm_dir)/doil/ORBManager.cpp
DataFlowComponentServantTests_SOURCES += $(open_rtm_dir)/doil/corba/CORBAManager.cpp

DataFlowComponentServantTests_LDFLAGS = -L$(libdir)

DataFlowComponentServantTests_LDADD   = -lomniORB4
DataFlowComponentServantTests_LDADD  += -lomnithread
DataFlowComponentServantTests_LDADD  += -lomniDynamic4
DataFlowComponentServantTests_LDADD  += -lcoil
#DataFlowComponentServantTests_LDADD  += -lcppunit
DataFlowComponentServantTests_LDADD  += $(target_srcdir)/DataFlowComponentServant.o
DataFlowComponentServantTests_LDADD  += $(target_srcdir)/RTCTypeConversion.o
DataFlowComponentServantTests_LDADD  += $(target_srcdir)/SDOPackageTypeConversion.o

DataFlowComponentServantTests_LDADD  += $(target_srcdir)/DataFlowComponentActionServant.o
DataFlowComponentServantTests_LDADD  += $(target_srcdir)/ComponentActionServant.o
DataFlowComponentServantTests_LDADD  += $(target_srcdir)/LightweightRTObjectServant.o

# all
all: do-test

# do tests
do-test:
	./DataFlowComponentServantTests

# clean-up
clean-local:
	rm -f                               \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl

