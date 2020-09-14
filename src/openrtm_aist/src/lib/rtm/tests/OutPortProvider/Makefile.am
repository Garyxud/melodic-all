#
# $Log: Makefile.am,v $
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS =                    \
	 -I.                      \
	-I$(top_builddir)         \
	-I$(top_builddir)/src/lib \
	-I$(top_builddir)/src/lib/rtm/idl \
	-I$(top_builddir)/src/lib/coil/include \
	-I$(top_builddir)/src/lib/rtm \
	-I$(includedir)

AM_LDFLAGS =              \
	-L.               \
	-L$(top_builddir) \
	-L$(top_builddir)/src/lib/coil/lib


IDL_SOURCES =                                \
	$(top_srcdir)/src/lib/rtm/idl/RTC.idl        \
	$(top_srcdir)/src/lib/rtm/idl/SDOPackage.idl \
	$(top_srcdir)/src/lib/rtm/idl/OpenRTM.idl \
	$(top_srcdir)/src/lib/rtm/idl/Manager.idl \
	$(top_srcdir)/src/lib/rtm/idl/BasicDataType.idl \
	$(top_srcdir)/src/lib/rtm/idl/DataPort.idl

noinst_PROGRAMS = OutPortProviderTests

nodist_OutPortProviderTests_SOURCES =             \
	../TestRunner.cpp                \
	$(IDL_SOURCES:.idl=Skel.cpp)     \
	$(top_srcdir)/src/lib/coil/common/stringutil.cpp \
	$(top_srcdir)/src/lib/coil/common/Properties.cpp \
	$(top_srcdir)/src/lib/rtm/NVUtil.cpp     \
	$(top_srcdir)/src/lib/rtm/OutPortProvider.cpp \
	OutPortProviderTests.cpp

OutPortProviderTests_LDFLAGS = -L$(libdir) $(AM_LDFLAGS)
OutPortProviderTests_LDADD   = -lcppunit -lcoil

EXTRA_DIST =        \
	OutPortProviderTests.cpp

# all
all: do-test

# do tests
do-test:
	./OutPortProviderTests --xml outportprovidertests.xml \
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
