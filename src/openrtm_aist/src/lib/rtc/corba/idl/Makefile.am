## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for libRTCSkel
##
## $Id$
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@
AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/rtc/corba
AM_LDFLAGS=-L$(top_builddir)/src/lib/rtc -L$(top_builddir)/src/lib/rtc/corba/idl

# Suffix definition of sources and headers of skeletons and stubs 
SUFFIXES = .idl Skel.cpp Stub.cpp Stub.o Skel.o

# RTM idl sources
IDL_SOURCES =             \
	SDOPackage.idl    \
	RTC.idl           \
	OpenRTM.idl       \
	BasicDataType.idl \
	DataPort.idl

SKEL_H   = $(IDL_SOURCES:.idl=Skel.h)
SKEL_CPP = $(IDL_SOURCES:.idl=Skel.cpp)
STUB_H   = $(IDL_SOURCES:.idl=Stub.h)
STUB_CPP = $(IDL_SOURCES:.idl=Stub.cpp)
ALL_SRC =           \
	$(SKEL_H)   \
	$(SKEL_CPP) \
	$(STUB_H)   \
	$(STUB_CPP) \
	$(ORB_SRC)

if ORB_IS_TAO
ORB_SRC = $(TAO_SRC)
endif

if ORB_IS_OMNIORB
ORB_SRC = $(OMNIORB_SRC)
endif

if ORB_IS_MICO
ORB_SRC = $(MICO_SRC)
endif

if ORB_IS_ORBIX
ORB_SRC = $(ORBIX_SRC)
endif

if ORB_IS_ORBACUS
ORB_SRC = $(ORBACUS_SRC)
endif

if ORB_IS_ORBIT2
ORB_SRC = $(ORBIT2_SRC)
OIDLC = $(IDLC) -DORBIT2_IDL
else
OIDLC = echo
endif


TAO_SRC = \
	$(IDL_SOURCES:.idl=S.h) \
	$(IDL_SOURCES:.idl=S.cpp) \
	$(IDL_SOURCES:.idl=S.inl) \
	$(IDL_SOURCES:.idl=S_T.cpp) \
	$(IDL_SOURCES:.idl=S_T.h) \
	$(IDL_SOURCES:.idl=S_T.inl) \
	$(IDL_SOURCES:.idl=C.h) \
	$(IDL_SOURCES:.idl=C.cpp) \
	$(IDL_SOURCES:.idl=C.inl)

OMNIORB_SRC = \
	$(IDL_SOURCES:.idl=.hh) \
	$(IDL_SOURCES:.idl=SK.cc) \
	$(IDL_SOURCES:.idl=DynSK.cc)

MICO_SRC = \
	$(IDL_SOURCES:.idl=.h) \
	$(IDL_SOURCES:.idl=.cc) \
	$(IDL_SOURCES:.idl=_skel.cc)

ORBIX_SRC = \
	$(IDL_SOURCES:.idl=.hh) \
	$(IDL_SOURCES:.idl=S.cc) \
	$(IDL_SOURCES:.idl=C.cc)

ORBACUS_SRC = \
	$(IDL_SOURCES:.idl=.h) \
	$(IDL_SOURCES:.idl=.cpp) \
	$(IDL_SOURCES:.idl=_skel.h) \
	$(IDL_SOURCES:.idl=_skel.cpp)

ORBIT2_SRC = \
	$(IDL_SOURCES:.idl=.h) \
	$(IDL_SOURCES:.idl=-common.c) \
	$(IDL_SOURCES:.idl=-cpp-common.cc) \
	$(IDL_SOURCES:.idl=-cpp-common.h) \
	$(IDL_SOURCES:.idl=-cpp-skels.cc) \
	$(IDL_SOURCES:.idl=-cpp-skels.h) \
	$(IDL_SOURCES:.idl=-cpp-stubs.cc) \
	$(IDL_SOURCES:.idl=-cpp-stubs.h) \
	$(IDL_SOURCES:.idl=-cpp.cc) \
	$(IDL_SOURCES:.idl=-cpp.h)


# Install directory of headers and idl files
rtmheaderdir = $(prefix)/include/rtm
idldir =       $(rtmheaderdir)/idl
rtmskeldir =   $(rtmheaderdir)/idl

idl_DATA = $(IDL_SOURCES)

nodist_rtmskel_HEADERS = $(ALL_SRC)


#------------------------------------------------------------
# Intermediate library : libRTCSkel.a
noinst_LTLIBRARIES = libRTCSkel.la libRTCStub.la

nodist_libRTCSkel_la_SOURCES = $(SKEL_CPP)

libRTCSkel_la_LIBADD = $(LIBS)
libRTCSkel_la_LDFLAGS = -rpath `pwd`

nodist_libRTCStub_la_SOURCES = $(STUB_CPP)
libRTCStub_la_LIBADD = $(LIBS)
libRTCStub_la_LDFLAGS = -rpath `pwd`

.idlSkel.cpp:
	$(OIDLC) $<
	$(IDLC) $(IDLFLAGS) $<
	$(top_srcdir)/build/makewrapper.py $< rtc/corba/idl
.idlStub.cpp:
	$(OIDLC) $<
	$(IDLC) $(IDLFLAGS) $<
	$(top_srcdir)/build/makewrapper.py $< rtc/corba/idl
#------------------------------------------------------------


#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(ALL_SRC) > skelstub.lst
	echo $(IDL_SOURCES) > idls.lst

#------------------------------------------------------------
# Visual Studio Project
#------------------------------------------------------------
win32_builddir=$(top_builddir)/win32/OpenRTM-aist/

vcproj: vc8proj vc9proj  vc10proj

vc8proj: libRTCSkel.vcproj.yaml
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type LIB \
		--vcversion "8.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc8.vcproj \
		--yaml libRTCSkel.vcproj.yaml \
		--source $(SKEL_CPP) \
		--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc8.vcproj

vc9proj: libRTCSkel.vcproj.yaml
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type LIB \
		--vcversion "9.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc9.vcproj \
		--yaml libRTCSkel.vcproj.yaml \
		--source $(SKEL_CPP) \
		--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc9.vcproj

vc10proj: libRTCSkel.vcproj.yaml
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type LIB \
		--vcversion "10.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc10.vcproj \
		--yaml libRTCSkel.vcproj.yaml \
		--source $(SKEL_CPP) \
		--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc10.vcproj

#------------------------------------------------------------
# for wxs file
#------------------------------------------------------------
wxs:
	$(top_srcdir)/build/makewxs.py flist \
		-c IDLs \
		-o $(win32_builddir)installer/idls.yaml \
		-p ..\\rtm\\idl \
		$(IDL_SOURCES) $(ALL_SRC)

dist-hook: lst vcproj wxs

# Cleanup
clean-local:
	rm -f $(ALL_SRC) *~ *.core *.lst Makefile.old

# Extra sources included distribution
EXTRA_DIST = \
	$(IDL_SOURCES) \
	libRTCSkel.vcproj.yaml



