# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Manager unit test
# @date   $Date: 2008/05/09 12:00:42 $
#
# $Id: Makefile.am,v 1.1 2008/05/09 12:00:42 arafune Exp $
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
	-I$(includedir)

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

noinst_LTLIBRARIES = DummyLib.la DummyModule.la DummyModule2.la  
nodist_DummyLib_la_SOURCES = \
	$(IDL_SOURCES:.idl=Skel.cpp) \
	$(top_srcdir)/src/lib/rtm/NVUtil.cpp \
	$(top_srcdir)/src/lib/rtm/CorbaNaming.cpp \
	$(top_srcdir)/src/lib/rtm/NamingManager.cpp \
	$(top_srcdir)/src/lib/rtm/PortBase.cpp \
	$(top_srcdir)/src/lib/rtm/PortAdmin.cpp \
	$(top_srcdir)/src/lib/rtm/ConfigAdmin.cpp \
	$(top_srcdir)/src/lib/rtm/CorbaPort.cpp          \
	$(top_srcdir)/src/lib/rtm/ConnectorListener.cpp  \
	$(top_srcdir)/src/lib/rtm/CORBA_IORUtil.cpp      \
	$(top_srcdir)/src/lib/rtm/SdoConfiguration.cpp \
	$(top_srcdir)/src/lib/rtm/ModuleManager.cpp \
	$(top_srcdir)/src/lib/rtm/ManagerConfig.cpp \
	$(top_srcdir)/src/lib/rtm/RTCUtil.cpp \
	$(top_srcdir)/src/lib/rtm/NumberingPolicy.cpp \
	$(top_srcdir)/src/lib/rtm/Factory.cpp \
	$(top_srcdir)/src/lib/rtm/ECFactory.cpp \
	$(top_srcdir)/src/lib/rtm/ExtTrigExecutionContext.cpp \
	$(top_srcdir)/src/lib/rtm/PeriodicExecutionContext.cpp \
	$(top_srcdir)/src/lib/rtm/OpenHRPExecutionContext.cpp \
	$(top_srcdir)/src/lib/rtm/RTObject.cpp \
	$(top_srcdir)/src/lib/rtm/Manager.cpp \
	$(top_srcdir)/src/lib/rtm/ManagerServant.cpp \
	$(top_srcdir)/src/lib/rtm/PeriodicECSharedComposite.cpp \
	$(top_srcdir)/src/lib/rtm/SdoOrganization.cpp \
	$(top_srcdir)/src/lib/rtm/SystemLogger.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortBase.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortProvider.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortConnector.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortPullConnector.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortPushConnector.cpp \
	$(top_srcdir)/src/lib/rtm/InPortBase.cpp \
	$(top_srcdir)/src/lib/rtm/InPortProvider.cpp \
	$(top_srcdir)/src/lib/rtm/InPortConnector.cpp \
	$(top_srcdir)/src/lib/rtm/InPortPullConnector.cpp \
	$(top_srcdir)/src/lib/rtm/InPortPushConnector.cpp \
	$(top_srcdir)/src/lib/rtm/FactoryInit.cpp \
	$(top_srcdir)/src/lib/rtm/CdrRingBuffer.cpp \
	$(top_srcdir)/src/lib/rtm/DefaultPeriodicTask.cpp \
	$(top_srcdir)/src/lib/rtm/PublisherFlush.cpp \
	$(top_srcdir)/src/lib/rtm/PublisherNew.cpp \
	$(top_srcdir)/src/lib/rtm/PublisherPeriodic.cpp \
	$(top_srcdir)/src/lib/rtm/InPortCorbaCdrProvider.cpp \
	$(top_srcdir)/src/lib/rtm/InPortCorbaCdrConsumer.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortCorbaCdrConsumer.cpp \
	$(top_srcdir)/src/lib/rtm/OutPortCorbaCdrProvider.cpp 
DummyLib_la_LDFLAGS = -module \
        -L$(top_builddir)/src/lib/coil/lib
DummyLib_la_LIBADD   =                            \
	-lomniORB4                                \
	-lomnithread                              \
	-lomniDynamic4                            \
	-lcoil                                    \
	-luuid

#
#
#
noinst_PROGRAMS = ManagerTests

nodist_ManagerTests_SOURCES = \
	../TestRunner.cpp \
	ManagerTests.cpp
ManagerTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS) \
        -L$(top_builddir)/src/lib/coil/lib
ManagerTests_LDADD   = \
         -lcppunit                                 \
         -lomniORB4                                \
         -lomnithread                              \
         -lomniDynamic4                            \
	-lcoil                                    \
         -luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/Manager/.libs/DummyLib.la



#
#
#
#socompdir  = $(pkgdatadir)/src/lib/rtm/tests/Manager

# Loadable RTCs
#socomp_LTLIBRARIES = ConsoleIn.la ConsoleOut.la
#noinst_LTLIBRARIES = DummyModule.la  DummyModule2.la  

nodist_DummyModule_la_SOURCES = DummyModule.h \
				$(top_srcdir)/src/lib/rtm/DataFlowComponentBase.cpp \
				DummyModule.cpp
DummyModule_la_LDFLAGS = -module \
	-L$(top_builddir)/src/lib/coil/lib
DummyModule_la_LIBADD  = \
	-lomniORB4                                \
	-lomnithread                              \
	-lomniDynamic4                            \
	-lcoil                                    \
	-luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/Manager/.libs/DummyLib.la

nodist_DummyModule2_la_SOURCES = DummyModule2.h \
				$(top_srcdir)/src/lib/rtm/DataFlowComponentBase.cpp \
				DummyModule2.cpp
DummyModule2_la_LDFLAGS = -module \
	-L$(top_builddir)/src/lib/coil/lib
DummyModule2_la_LIBADD  = \
	-lomniORB4                                \
	-lomnithread                              \
	-lomniDynamic4                            \
	-lcoil                                    \
	-luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/Manager/.libs/DummyLib.la

bin_PROGRAMS = rtcprof
rtcprof_SOURCES = $(top_builddir)/utils/rtcprof/rtcprof.cpp
rtcprof_LDADD   = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
rtcprof_LDFLAGS = -L$(top_builddir)/src/lib/rtm



EXTRA_DIST = \
	ManagerTests.cpp \
	DummyModule.h \
	DummyModule.cpp \
	DummyModule2.h \
	DummyModule2.cpp \
	fixture1.conf \
	fixture2.conf \
	fixture3.conf \
	fixture4.conf


# all
all: do-test

# do tests
do-test:
	./ManagerTests --xml managertests.xml --xsl default.xsl

# clean-up
clean-local:
	rm -f                               \
        *.log *.xml                         \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl
