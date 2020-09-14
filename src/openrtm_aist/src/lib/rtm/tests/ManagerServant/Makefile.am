# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for ManagerServant unit test
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

AM_CPPFLAGS= -I.                                \
	-I$(includedir)                         \
	-I$(top_builddir)                       \
	-I.                                     \
	-I../                                   \
        -I$(top_builddir)/src/lib/              \
        -I$(top_builddir)/src/lib/rtm           \
        -I$(top_builddir)/src/lib/coil/include  \
        -I$(top_builddir)/src/lib/rtm/idl       \
        -I$(top_builddir)/src/lib/rtm           \
        -I/usr/include/                         \
        -I/usr/include/omniORB4/

AM_LDFLAGS= -L.                            \
	-L$(top_builddir)                  \
        -L$(top_builddir)/src/lib/coil/lib


IDL_SOURCES =                                                   \
        $(top_builddir)/src/lib/rtm/idl/SDOPackage.idl    \
        $(top_builddir)/src/lib/rtm/idl/RTC.idl           \
        $(top_builddir)/src/lib/rtm/idl/OpenRTM.idl       \
        $(top_builddir)/src/lib/rtm/idl/BasicDataType.idl \
        $(top_builddir)/src/lib/rtm/idl/DataPort.idl      \
        $(top_builddir)/src/lib/rtm/idl/Manager.idl

#dummylibdir  = $(pkgdatadir)/src/lib/rtm/tests/ManagerServant
noinst_LTLIBRARIES = DummyLib.la DummyModule1.la DummyModule2.la
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
        $(top_builddir)/src/lib/rtm/ManagerServant.cpp
DummyLib_la_LDFLAGS = -module \
        -L$(top_builddir)/src/lib/coil/lib
DummyLib_la_LIBADD   =                            \
	-lomniORB4                                \
	-lomnithread                              \
	-lomniDynamic4                            \
	-lcoil                                    \
	-luuid


noinst_PROGRAMS = ManagerServantTests

nodist_ManagerServantTests_SOURCES =                             \
        ../TestRunner.cpp                                        \
        ManagerServantTests.cpp
ManagerServantTests_LDFLAGS = -L$(libdir) \
        -L$(top_builddir)/src/lib/coil/lib
ManagerServantTests_LDADD   =                      \
         -lcppunit                                 \
         -lomniORB4                                \
         -lomnithread                              \
         -lomniDynamic4                            \
         -lcoil                                    \
         -luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/ManagerServant/.libs/DummyLib.la

TEST_SRC = $(ManagerServantTests_SOURCES)
TEST_H   = 


#socompdir  = $(pkgdatadir)/src/lib/rtm/tests/ManagerServant

# Loadable RTCs
#socomp_LTLIBRARIES = ConsoleIn.la ConsoleOut.la
#noinst_LTLIBRARIES = DummyModule1.la DummyModule2.la

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
	$(top_builddir)/src/lib/rtm/tests/ManagerServant/.libs/DummyLib.la

nodist_DummyModule2_la_SOURCES = DummyModule2.h \
	$(top_builddir)/src/lib/rtm/DataFlowComponentBase.cpp    \
                        DummyModule2.cpp
DummyModule2_la_LDFLAGS = -module \
        -L$(top_builddir)/src/lib/coil/lib
DummyModule2_la_LIBADD  = \
	-lomniORB4                                \
	-lomnithread                              \
	-lomniDynamic4                            \
	-lcoil                                    \
	-luuid                                    \
	$(top_builddir)/src/lib/rtm/tests/ManagerServant/.libs/DummyLib.la


bin_PROGRAMS = rtcprof
rtcprof_SOURCES = $(top_builddir)/utils/rtcprof/rtcprof.cpp
rtcprof_LDADD   = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
rtcprof_LDFLAGS = -L$(top_builddir)/src/lib/rtm


EXTRA_DIST = \
	ManagerServantTests.cpp \
	DummyModule1.h \
	DummyModule1.cpp \
	DummyModule2.h \
	DummyModule2.cpp


# all
all: do-test

# do tests
do-test:
#	./ManagerServantTests
	./ManagerServantTests --xml managerservanttests.xml \
	--xsl default.xsl

# clean-up
clean-local:
	rm -f *.o *.Po *.gch *.la
	rm -f *~ *core
	rm -f *.xml
	rm -f Makefile.old
	rm -f *.vcproj
	rm -rf Release Debug
	rm -f *.log
#	rm -f *.lo
#	rm -rf .deps 

#------------------------------------------------------------
# vcproj file build rules
#------------------------------------------------------------
win32_builddir = .

vcproj: ManagerServant_vc8.vcproj ManagerServant_vc9.vcproj ManagerServant_vc10.vcproj

ManagerServant_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname ManagerServantTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/ManagerServant_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/ManagerServant_vc8.vcproj

ManagerServant_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname ManagerServantTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/ManagerServant_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/ManagerServant_vc9.vcproj

ManagerServant_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname ManagerServantTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/ManagerServant_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/ManagerServant_vc10.vcproj


