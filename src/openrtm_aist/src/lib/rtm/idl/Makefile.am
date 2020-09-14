## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for libRTCSkel
##
## $Id$
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

SUBDIRS = . device_interfaces

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@
AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/rtm
AM_LDFLAGS=-L$(top_builddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

# RTM idl sources
if ORB_IS_RTORB
IDL_SOURCES = OpenRTM-aist.idl
else
IDL_SOURCES =             \
	SDOPackage.idl        \
	RTC.idl               \
	OpenRTM.idl           \
	Manager.idl           \
	BasicDataType.idl     \
	DataPort.idl          \
	ExtendedDataTypes.idl \
	InterfaceDataTypes.idl
endif

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

# RtORB cannot handle multiple IDL files
# Unified IDL OpenRTM-aist.idl is compiled and linked.
# Other skel/stub headers are provided as dummies.
if ORB_IS_RTORB
DUMMY_SKELS =                   \
        SDOPackageSkel.h        \
        RTCSkel.h               \
        OpenRTMSkel.h           \
        ManagerSkel.h           \
        BasicDataTypeSkel.h     \
        DataPortSkel.h          \
        ExtendedDataTypesSkel.h \
        InterfaceDataTypesSkel.h
ORB_SRC = $(RTORB_SRC)
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

RTORB_SRC = \
	OpenRTM-aist.h          \
	OpenRTM-aist-common.c   \
	OpenRTM-aist-skelimpl.c \
	OpenRTM-aist-skels.c    \
	OpenRTM-aist-stubs.c

# This IDL file is for only RtORB
OpenRTM-aist.idl:
	echo '// This IDL file is for only RtORB' > OpenRTM-aist.idl
	echo '#include "BasicDataType.idl"' >> OpenRTM-aist.idl
	echo '#include "ExtendedDataTypes.idl"' >> OpenRTM-aist.idl
	echo '#include "RTC.idl"' >> OpenRTM-aist.idl
	echo '#include "OpenRTM.idl"' >> OpenRTM-aist.idl
	echo '#pragma ""' >> OpenRTM-aist.idl
	echo '#include "Manager.idl"' >> OpenRTM-aist.idl
	echo '#include "DataPort.idl"' >> OpenRTM-aist.idl
	echo '#include "InterfaceDataTypes.idl"' >> OpenRTM-aist.idl

idl-stamp: $(IDL_SOURCES)
	@touch idl-stamp
	@for f in $(IDL_SOURCES) ; do \
		$(OIDLC) $$f ; \
		$(IDLC) $(IDLFLAGS) $$f ; \
		$(top_srcdir)/build/makewrapper.py $$f ; \
	done ;

$(ALL_SRC): idl-stamp
	@if test -f $@ ; then \
		touch $@ ; \
	else \
		rm -f idl-stamp ; \
		$(MAKE) $(AM_MAKEFLAGS) idl-stamp ; \
	fi ;

OpenRTM-aistSkel.cpp OpenRTM-aistStub.cpp: OpenRTM-aist.idl
	@for f in $(IDL_SOURCES) ; do \
		$(OIDLC) $$f ; \
		$(IDLC) $(IDLFLAGS) $$f ; \
		$(top_srcdir)/build/makewrapper.py $$f ; \
	done ;
	@for f in $(DUMMY_SKELS) ; do \
		cp OpenRTM-aistSkel.h $$f ; \
	done


# Install directory of headers and idl files
rtmheaderdir = $(rtm_includedir)/rtm
idldir =       $(rtmheaderdir)/idl
rtmskeldir =   $(rtmheaderdir)/idl

idl_DATA = $(IDL_SOURCES)

nodist_rtmskel_HEADERS = $(ALL_SRC) 

#------------------------------------------------------------
# Intermediate library : libRTCSkel.a
noinst_LTLIBRARIES = libRTCSkel.la libRTCStub.la

if ORB_IS_RTORB
nodist_libRTCSkel_la_SOURCES = OpenRTM-aistSkel.cpp
nodist_libRTCStub_la_SOURCES = OpenRTM-aistStub.cpp
else
nodist_libRTCSkel_la_SOURCES = $(SKEL_CPP)
nodist_libRTCStub_la_SOURCES = $(STUB_CPP)
endif

libRTCSkel_la_LIBADD = $(LIBS)
libRTCStub_la_LIBADD = $(LIBS)
#
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

vcproj: vc8proj vc9proj vc10proj vc11proj vc12proj vc14proj

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
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type LIB \
		--vcversion "10.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc10.vcxproj \
		--yaml libRTCSkel.vcproj.yaml \
		--source $(SKEL_CPP) \
		--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc10.vcxproj

vc11proj: libRTCSkel.vcproj.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type LIB \
		--vcversion "11.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc11.vcxproj \
		--yaml libRTCSkel.vcproj.yaml \
		--source $(SKEL_CPP) \
		--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc11.vcxproj

vc12proj: libRTCSkel.vcproj.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type LIB \
		--vcversion "12.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/idl/libRTCSkel_vc12.vcxproj \
		--yaml libRTCSkel.vcproj.yaml \
		--source $(SKEL_CPP) \
		--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc12.vcxproj
vc14proj: libRTCSkel.vcproj.yaml
		$(top_builddir)/build/vcxprojtool.py vcxproj \
			--type LIB \
			--vcversion "14.00" \
			--version $(RTM_VERSION) \
			--out $(win32_builddir)/rtm/idl/libRTCSkel_vc14.vcxproj \
			--yaml libRTCSkel.vcproj.yaml \
			--source $(SKEL_CPP) \
			--header $(SKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/idl/libRTCSkel_vc14.vcxproj
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
	rm -f $(ALL_SRC) *~ *.core *.lst Makefile.old *.h *.c
	rm -f idl-stamp

# Extra sources included distribution
EXTRA_DIST = \
	$(IDL_SOURCES) \
	libRTCSkel.vcproj.yaml

DIST_SUBDIRS = device_interfaces
