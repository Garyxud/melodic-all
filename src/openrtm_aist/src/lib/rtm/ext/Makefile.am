## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for libRTCSkel
##
## $Id: Makefile.am 2697 2016-01-18 00:47:39Z kawauchi $
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@ -I$(top_srcdir)/src/lib/rtm/idl
LIBS = @LIBS@
AM_CPPFLAGS =                       \
	-I$(top_srcdir)/src/lib         \
	-I$(top_srcdir)/src/lib/rtm     \
	-I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS =                        \
	-L$(top_builddir)/src/lib/rtm   \
	-L$(top_builddir)/src/lib/rtm/idl

MANIPULATOR_IDLS =                            \
	ManipulatorCommonInterface_DataTypes.idl  \
	ManipulatorCommonInterface_Common.idl     \
	ManipulatorCommonInterface_Middle.idl
CAMERA_IDLS = CameraCommonInterface.idl

# RTM idl sources
IDL_SOURCES =                      \
	$(MANIPULATOR_IDLS)            \
	$(CAMERA_IDLS)


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

#------------------------------------------------------------
# extended libraries
#------------------------------------------------------------
MANIPULATORSKEL_H   = $(MANIPULATOR_IDLS:.idl=Skel.h)
MANIPULATORSKEL_CPP = $(MANIPULATOR_IDLS:.idl=Skel.cpp)
MANIPULATORSTUB_H   = $(MANIPULATOR_IDLS:.idl=Stub.h)
MANIPULATORSTUB_CPP = $(MANIPULATOR_IDLS:.idl=Stub.cpp)

CAMERASKEL_H   = $(CAMERA_IDLS:.idl=Skel.h)
CAMERASKEL_CPP = $(CAMERA_IDLS:.idl=Skel.cpp)
CAMERASTUB_H   = $(CAMERA_IDLS:.idl=Stub.h)
CAMERASTUB_CPP = $(CAMERA_IDLS:.idl=Stub.cpp)

WRAPPER  = $(top_builddir)/utils/rtm-skelwrapper/rtm-skelwrapper
WRAPPER_FLAGS = --include-dir="" --skel-suffix=Skel --stub-suffix=Stub

idl-stamp: $(IDL_SOURCES)
	@touch idl-stamp
	@for f in $(IDL_SOURCES) ; do \
		$(OIDLC) $$f ; \
		$(IDLC) $(IDLFLAGS) $$f ; \
		PYTHONPATH="$(top_srcdir)/build:$(PYTHONPATH)" \
		PATH="$(top_srcdir)/utils/rtm-config:$(PATH)"  \
		$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=$$f ;   \
	done ;

$(ALL_SRC): idl-stamp
	@if test -f $@ ; then \
		touch $@ ; \
	else \
		rm -f idl-stamp ; \
		$(MAKE) $(AM_MAKEFLAGS) idl-stamp ; \
	fi ;

#------------------------------------------------------------
# Install directory of headers and idl files
#------------------------------------------------------------
rtmheaderdir = $(rtm_includedir)/rtm
idldir =       $(rtmheaderdir)/ext
extskeldir =   $(rtmheaderdir)/ext

idl_DATA = $(IDL_SOURCES)
nodist_extskel_HEADERS = $(ALL_SRC)

#------------------------------------------------------------
# libraries
#------------------------------------------------------------
lib_LTLIBRARIES = \
	librtmManipulator.la \
	librtmCamera.la

nodist_librtmManipulator_la_SOURCES = $(MANIPULATORSKEL_CPP)
nodist_librtmCamera_la_SOURCES = $(CAMERASKEL_CPP)

librtmManipulator_la_LIBADD = $(LIBS) -lRTC
librtmCamera_la_LIBADD = $(LIBS) -lRTC

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

vcproj: vc9proj vc10proj vc11proj vc12proj vc14proj

vc9proj: librtmManipulator.vcproj.yaml librtmCamera.vcproj.yaml
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type DLL \
		--vcversion "9.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmManipulator_vc9.vcproj \
		--yaml librtmManipulator.vcproj.yaml \
		--source $(MANIPULATORSKEL_CPP) \
		--header $(MANIPULATORSKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmManipulator_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type DLL \
		--vcversion "9.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmCamera_vc9.vcproj \
		--yaml librtmCamera.vcproj.yaml \
		--source $(CAMERASKEL_CPP) \
		--header $(CAMERASKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmCamera_vc9.vcproj

vc10proj: librtmManipulator.vcproj.yaml librtmCamera.vcproj.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "10.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmManipulator_vc10.vcxproj \
		--yaml librtmManipulator.vcproj.yaml \
		--source $(MANIPULATORSKEL_CPP) \
		--header $(MANIPULATORSKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmManipulator_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "10.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmCamera_vc10.vcxproj \
		--yaml librtmCamera.vcproj.yaml \
		--source $(CAMERASKEL_CPP) \
		--header $(CAMERASKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmCamera_vc10.vcxproj

vc11proj: librtmManipulator.vcproj.yaml librtmCamera.vcproj.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "11.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmManipulator_vc11.vcxproj \
		--yaml librtmManipulator.vcproj.yaml \
		--source $(MANIPULATORSKEL_CPP) \
		--header $(MANIPULATORSKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmManipulator_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "11.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmCamera_vc11.vcxproj \
		--yaml librtmCamera.vcproj.yaml \
		--source $(CAMERASKEL_CPP) \
		--header $(CAMERASKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmCamera_vc11.vcxproj

vc12proj: librtmManipulator.vcproj.yaml librtmCamera.vcproj.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "12.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmManipulator_vc12.vcxproj \
		--yaml librtmManipulator.vcproj.yaml \
		--source $(MANIPULATORSKEL_CPP) \
		--header $(MANIPULATORSKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmManipulator_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "12.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmCamera_vc12.vcxproj \
		--yaml librtmCamera.vcproj.yaml \
		--source $(CAMERASKEL_CPP) \
		--header $(CAMERASKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmCamera_vc12.vcxproj

vc14proj: librtmManipulator.vcproj.yaml librtmCamera.vcproj.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "14.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmManipulator_vc14.vcxproj \
		--yaml librtmManipulator.vcproj.yaml \
		--source $(MANIPULATORSKEL_CPP) \
		--header $(MANIPULATORSKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmManipulator_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "14.00" \
		--version $(RTM_VERSION) \
		--out $(win32_builddir)/rtm/ext/librtmCamera_vc14.vcxproj \
		--yaml librtmCamera.vcproj.yaml \
		--source $(CAMERASKEL_CPP) \
		--header $(CAMERASKEL_H)
	qkc -O- -sm $(win32_builddir)/rtm/ext/librtmCamera_vc14.vcxproj


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
	rm -f OpenRTM-aist.idl
	rm -f idl-stamp

# Extra sources included distribution
EXTRA_DIST =                      \
	$(IDL_SOURCES)                \
	librtmManipulator.vcproj.yaml \
	librtmCamera.vcproj.yaml
