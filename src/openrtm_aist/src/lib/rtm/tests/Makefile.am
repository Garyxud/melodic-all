## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for unit test
##
## $Id$
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

SUBDIRS = \
	CORBA_IORUtil     \
	CORBA_SeqUtil     \
	ConfigAdmin \
	ConnectorListener \
	CorbaConsumer     \
	CorbaNaming \
	CorbaPort         \
	DataInOutPort     \
	ECFactory \
	ExtTrigExecutionContext \
	Factory           \
	InPort            \
	InPortBase \
	InPortConnector \
	InPortPushConnector \
	InPortPullConnector \
	InPortCorbaCdrConsumer \
	InPortCorbaCdrProvider \
	InPortProvider \
	Manager  \
	ManagerConfig     \
	ManagerServant \
	ModuleManager     \
	NVUtil            \
	NamingManager \
	NumberingPolicy \
	ObjectManager     \
	OutPort           \
	OutPortBase       \
	OutPortConnector \
	OutPortPushConnector \
	OutPortPullConnector \
	OutPortCorbaCdrConsumer \
	OutPortCorbaCdrProvider \
	OutPortProvider   \
	PeriodicECSharedComposite \
	PeriodicExecutionContext \
	PortAdmin         \
	PortBase          \
	PublisherFlush    \
	PublisherNew      \
	PublisherPeriodic \
	RTCUtil \
	RTObject \
	RingBuffer        \
	SdoConfiguration  \
	SdoOrganization  \
	SdoService  \
	StateMachine \
	SystemLogger

DIST_SUBDIRS = $(SUBDIRS)
#DIST_SUBDIRS += InPortTcpSockConsumer
#DIST_SUBDIRS += InPortTcpSockProvider

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS= -I.          \
	-I$(top_builddir) \
	-I$(includedir)   \
	-I$(top_builddir)/rtm/idl

AM_LDFLAGS=                       \
	-L$(top_builddir)         \
	-L$(top_builddir)/rtm     \
	-L$(top_builddir)/rtm/idl

# all
all: do-test

# do tests
do-test:
#	./AllTests --xml alltests.xml --xsl default.xsl

# Cleanup
clean-local:
	rm -f alltests.xml new.conf stored.conf \
        *.log *.xml                         \
	*.o *.Po *.gch *.la \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core \
	Makefile.old \
	*.cc *.hh *.i *.a *.c *.inl
