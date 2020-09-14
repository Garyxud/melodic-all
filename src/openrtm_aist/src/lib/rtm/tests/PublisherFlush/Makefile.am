# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for PublisherFlush unit test
# @date   $Date: 2007/01/21 06:53:09 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

#
# $Log: Makefile.am,v $
# Revision 1.3  2007/01/21 06:53:09  n-ando
# The include search path option order in AM_CPPFLAGS was changed.
#
# Revision 1.2  2007/01/12 14:45:00  n-ando
# Some fixes for distribution control.
#
# Revision 1.1  2006/12/18 06:51:03  n-ando
# The first commitment.
#
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS =             \
	-I.               \
	-I$(top_builddir) \
	-I$(top_builddir)/src/lib/ \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/rtm \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(includedir)


AM_LDFLAGS =              \
	-L.               \
	-L$(top_builddir) \
        -L$(top_builddir)/src/lib/coil/lib/ 

IDL_SOURCES = \
	$(top_srcdir)/src/lib/rtm/idl/SDOPackage.idl \
	$(top_srcdir)/src/lib/rtm/idl/RTC.idl \
	$(top_srcdir)/src/lib/rtm/idl/OpenRTM.idl \
	$(top_srcdir)/src/lib/rtm/idl/Manager.idl \
	$(top_srcdir)/src/lib/rtm/idl/BasicDataType.idl \
	$(top_srcdir)/src/lib/rtm/idl/DataPort.idl

noinst_PROGRAMS = PublisherFlushTests

nodist_PublisherFlushTests_SOURCES =           \
	../TestRunner.cpp                      \
	$(IDL_SOURCES:.idl=Skel.cpp) \
	$(top_srcdir)/src/lib/rtm/NVUtil.cpp \
	$(top_builddir)/src/lib/coil/common/Properties.cpp     \
	$(top_srcdir)/src/lib/rtm/SystemLogger.cpp           \
	$(top_srcdir)/src/lib/rtm/Manager.cpp                      \
	$(top_srcdir)/src/lib/rtm/NumberingPolicy.cpp              \
	$(top_srcdir)/src/lib/rtm/ModuleManager.cpp                \
	$(top_srcdir)/src/lib/rtm/ManagerConfig.cpp                \
	$(top_srcdir)/src/lib/rtm/ManagerServant.cpp               \
	$(top_srcdir)/src/lib/rtm/CorbaNaming.cpp                  \
	$(top_srcdir)/src/lib/rtm/CorbaPort.cpp          \
	$(top_srcdir)/src/lib/rtm/ConnectorListener.cpp  \
	$(top_srcdir)/src/lib/rtm/CORBA_IORUtil.cpp      \
	$(top_srcdir)/src/lib/rtm/NamingManager.cpp                \
	$(top_srcdir)/src/lib/rtm/ExtTrigExecutionContext.cpp      \
	$(top_srcdir)/src/lib/rtm/PeriodicExecutionContext.cpp     \
	$(top_srcdir)/src/lib/rtm/OpenHRPExecutionContext.cpp      \
	$(top_srcdir)/src/lib/rtm/PeriodicECSharedComposite.cpp    \
	$(top_srcdir)/src/lib/rtm/Factory.cpp                      \
	$(top_srcdir)/src/lib/rtm/ECFactory.cpp                    \
	$(top_srcdir)/src/lib/rtm/FactoryInit.cpp              \
	$(top_srcdir)/src/lib/rtm/CdrRingBuffer.cpp            \
	$(top_srcdir)/src/lib/rtm/DefaultPeriodicTask.cpp      \
	$(top_srcdir)/src/lib/rtm/RTObject.cpp         \
	$(top_srcdir)/src/lib/rtm/ConfigAdmin.cpp      \
	$(top_srcdir)/src/lib/rtm/PortAdmin.cpp        \
	$(top_srcdir)/src/lib/rtm/SdoOrganization.cpp  \
	$(top_srcdir)/src/lib/rtm/SdoConfiguration.cpp \
	$(top_builddir)/src/lib/rtm/PublisherFlush.cpp \
	$(top_srcdir)/src/lib/rtm/PortBase.cpp \
	$(top_srcdir)/src/lib/rtm/InPortBase.cpp                 \
	$(top_srcdir)/src/lib/rtm/OutPortBase.cpp                \
	$(top_srcdir)/src/lib/rtm/PublisherNew.cpp             \
	$(top_srcdir)/src/lib/rtm/PublisherPeriodic.cpp        \
	$(top_srcdir)/src/lib/rtm/InPortCorbaCdrProvider.cpp   \
	$(top_srcdir)/src/lib/rtm/InPortCorbaCdrConsumer.cpp   \
	$(top_srcdir)/src/lib/rtm/OutPortCorbaCdrConsumer.cpp  \
	$(top_srcdir)/src/lib/rtm/OutPortCorbaCdrProvider.cpp  \
	$(top_srcdir)/src/lib/rtm/OutPortProvider.cpp            \
	$(top_srcdir)/src/lib/rtm/OutPortConnector.cpp           \
	$(top_srcdir)/src/lib/rtm/OutPortPullConnector.cpp       \
	$(top_srcdir)/src/lib/rtm/OutPortPushConnector.cpp       \
	$(top_srcdir)/src/lib/rtm/InPortProvider.cpp             \
	$(top_srcdir)/src/lib/rtm/InPortConnector.cpp            \
	$(top_srcdir)/src/lib/rtm/InPortPullConnector.cpp        \
	$(top_srcdir)/src/lib/rtm/InPortPushConnector.cpp        \
	$(top_builddir)/src/lib/coil/common/stringutil.cpp     \
	PublisherFlushTests.cpp

PublisherFlushTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
PublisherFlushTests_LDADD   = -lcppunit -lcoil

EXTRA_DIST =                \
	PublisherFlushTests.cpp

# all
all: do-test

# do tests
do-test:
	./PublisherFlushTests --xml publisherflushtests.xml \
	--xsl default.xsl

# clean-up
clean-local:
	rm -f PublisherFlushTests           \
        *.log *.xml                         \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	*.cc *.hh *.i *.a *.c *.inl
