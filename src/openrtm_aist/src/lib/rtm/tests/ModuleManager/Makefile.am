# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ModuleManager unit test
# @date   $Date: 2007/01/21 06:52:17 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS = \
	 -I. \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(top_builddir)/src/lib/rtm \
	-I$(includedir) \
	-I$(top_builddir)/src/lib/coil/posix

AM_LDFLAGS = \
	 -L. \
	-L$(top_builddir) \
	-L$(top_builddir)/src/lib/coil/lib

IDL_SOURCES = \
	$(top_srcdir)/src/lib/rtm/idl/SDOPackage.idl \
	$(top_srcdir)/src/lib/rtm/idl/RTC.idl \
	$(top_srcdir)/src/lib/rtm/idl/OpenRTM.idl \
	$(top_srcdir)/src/lib/rtm/idl/Manager.idl \
	$(top_srcdir)/src/lib/rtm/idl/BasicDataType.idl \
	$(top_srcdir)/src/lib/rtm/idl/DataPort.idl

#dummylibdir  = $(pkgdatadir)/src/lib/rtm/tests/ModuleManager
noinst_LTLIBRARIES = DummyLib.la DummyModule1.la
nodist_DummyLib_la_SOURCES = \
	$(IDL_SOURCES:.idl=Skel.cpp)                             \
	$(top_builddir)/src/lib/rtm/NVUtil.cpp                   \
	$(top_builddir)/src/lib/rtm/CorbaNaming.cpp              \
	$(top_builddir)/src/lib/rtm/NamingManager.cpp            \
	$(top_builddir)/src/lib/rtm/PortBase.cpp                 \
	$(top_builddir)/src/lib/rtm/PortAdmin.cpp                \
	$(top_builddir)/src/lib/rtm/ConfigAdmin.cpp              \
	$(top_builddir)/src/lib/rtm/SdoOrganization.cpp          \
	$(top_builddir)/src/lib/rtm/SdoConfiguration.cpp         \
	$(top_builddir)/src/lib/rtm/ModuleManager.cpp            \
	$(top_builddir)/src/lib/rtm/ManagerConfig.cpp            \
	$(top_builddir)/src/lib/rtm/RTCUtil.cpp                  \
	$(top_builddir)/src/lib/rtm/NumberingPolicy.cpp          \
	$(top_builddir)/src/lib/rtm/Factory.cpp                  \
	$(top_builddir)/src/lib/rtm/ECFactory.cpp                \
	$(top_builddir)/src/lib/rtm/ExtTrigExecutionContext.cpp  \
	$(top_builddir)/src/lib/rtm/PeriodicExecutionContext.cpp \
	$(top_builddir)/src/lib/rtm/OpenHRPExecutionContext.cpp  \
	$(top_builddir)/src/lib/rtm/PeriodicECSharedComposite.cpp\
	$(top_builddir)/src/lib/rtm/RTObject.cpp                 \
	$(top_builddir)/src/lib/rtm/Manager.cpp                  \
	$(top_srcdir)/src/lib/rtm/CorbaPort.cpp               \
	$(top_srcdir)/src/lib/rtm/ConnectorListener.cpp       \
	$(top_srcdir)/src/lib/rtm/CORBA_IORUtil.cpp           \
	$(top_srcdir)/src/lib/rtm/SystemLogger.cpp               \
	$(top_srcdir)/src/lib/rtm/FactoryInit.cpp                  \
	$(top_srcdir)/src/lib/rtm/CdrRingBuffer.cpp                \
	$(top_srcdir)/src/lib/rtm/DefaultPeriodicTask.cpp          \
	$(top_srcdir)/src/lib/rtm/PublisherFlush.cpp               \
	$(top_srcdir)/src/lib/rtm/PublisherNew.cpp                 \
	$(top_srcdir)/src/lib/rtm/PublisherPeriodic.cpp            \
	$(top_srcdir)/src/lib/rtm/InPortCorbaCdrProvider.cpp       \
	$(top_srcdir)/src/lib/rtm/InPortCorbaCdrConsumer.cpp       \
	$(top_srcdir)/src/lib/rtm/OutPortCorbaCdrConsumer.cpp      \
	$(top_srcdir)/src/lib/rtm/OutPortCorbaCdrProvider.cpp      \
	$(top_srcdir)/src/lib/rtm/OutPortBase.cpp                \
	$(top_srcdir)/src/lib/rtm/OutPortProvider.cpp            \
	$(top_srcdir)/src/lib/rtm/OutPortConnector.cpp           \
	$(top_srcdir)/src/lib/rtm/OutPortPullConnector.cpp       \
	$(top_srcdir)/src/lib/rtm/OutPortPushConnector.cpp       \
	$(top_srcdir)/src/lib/rtm/InPortBase.cpp                 \
	$(top_srcdir)/src/lib/rtm/InPortProvider.cpp             \
	$(top_srcdir)/src/lib/rtm/InPortConnector.cpp            \
	$(top_srcdir)/src/lib/rtm/InPortPullConnector.cpp        \
	$(top_srcdir)/src/lib/rtm/InPortPushConnector.cpp        \
	$(top_builddir)/src/lib/rtm/DataFlowComponentBase.cpp    \
        $(top_builddir)/src/lib/rtm/ManagerServant.cpp
DummyLib_la_LDFLAGS = -module \
        -L$(top_builddir)/src/lib/coil/lib
DummyLib_la_LIBADD   =                            \
	-lomniORB4                                \
	-lomnithread                              \
	-lomniDynamic4                            \
	-lcoil                                    \
	-luuid

noinst_PROGRAMS = ModuleManagerTests

nodist_ModuleManagerTests_SOURCES = \
	../TestRunner.cpp \
	ModuleManagerTests.cpp
#ModuleManagerTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS) \
#        -L$(top_builddir)/src/lib/coil/lib
ModuleManagerTests_LDFLAGS = -L$(libdir) \
        -L$(top_builddir)/src/lib/coil/lib
ModuleManagerTests_LDADD   =                       \
         -lcppunit                                 \
         -lomniORB4                                \
         -lomnithread                              \
         -lomniDynamic4                            \
         -lcoil                                    \
         -luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/ModuleManager/.libs/DummyLib.la

TEST_SRC = $(ModuleManagerTests_SOURCES)
TEST_H   = 

#socompdir  = $(pkgdatadir)/src/lib/rtm/tests/ModuleManager

# Loadable RTCs
#noinst_LTLIBRARIES = DummyModule1.la

nodist_DummyModule1_la_SOURCES = DummyModule1.h \
	$(top_builddir)/src/lib/rtm/DataFlowComponentBase.cpp    \
                       DummyModule1.cpp 
DummyModule1_la_LDFLAGS = -module \
        -L$(top_builddir)/src/lib/coil/lib
DummyModule1_la_LIBADD  = \
         -lomniORB4                                \
         -lomnithread                              \
         -lomniDynamic4                            \
         -lcoil                                    \
         -luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/ModuleManager/.libs/DummyLib.la

bin_PROGRAMS = rtcprof
rtcprof_SOURCES = $(top_builddir)/utils/rtcprof/rtcprof.cpp
rtcprof_LDADD   = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
rtcprof_LDFLAGS = -L$(top_builddir)/src/lib/rtm

EXTRA_DIST = \
	ModuleManagerTests.cpp \
	DummyModule1.h \
	DummyModule1.cpp 

# all
all: do-test

# do tests
do-test:
	./ModuleManagerTests --xml modulemanagertests.xml \
	--xsl default.xsl

# clean-up
clean-local:
	rm -f                               \
        *.log *.xml                         \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl
