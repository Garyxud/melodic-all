## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for AutoTestIn/AutoTestOut components
##
## $Id: Makefile.am 1135 2009-01-19 16:11:02Z n-ando $
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

#------------------------------------------------------------
# Some rules for IDL compile
#------------------------------------------------------------
SUFFIXES = .o .so .idl Skel.cpp Stub.cpp Skel.o Stub.o

WRAPPER  = $(top_builddir)/utils/rtm-skelwrapper/rtm-skelwrapper
WRAPPER_FLAGS = --include-dir="" --skel-suffix=Skel --stub-suffix=Stub

.idlSkel.cpp:
	echo $(PROVIDER_OBJ)
	$(IDLC) $(IDLFLAGS) $<
	PYTHONPATH=$(top_srcdir)/build:$(PYTHONPATH) \
	PATH=$(top_srcdir)/utils/rtm-config:$(PATH)  \
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=$<

.idlStub.cpp:
	$(IDLC) $(IDLFLAGS) $<
	PYTHONPATH=$(PYTHONPATH):$(top_srcdir)/build \
	PATH=$(top_srcdir)/utils/rtm-config:$(PATH)  \
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=$<

#------------------------------------------------------------
# Source list
#------------------------------------------------------------
IDL_SOURCES = AutoTestService.idl

PROVIDER_DIST_SRC   = AutoTestServiceSVC_impl.cpp AutoTestIn.cpp
PROVIDER_NODIST_SRC = $(IDL_SOURCES:.idl=Skel.cpp)
PROVIDER_SRC        = $(PROVIDER_DIST_SRC) $(PROVIDER_NODIST_SRC)

PROVIDER_DIST_H     = $(PROVIDER_DIST_SRC:.cpp=.h)
PROVIDER_NODIST_H   = $(PROVIDER_NODIST_SRC:.cpp=.h)
PROVIDER_H          = $(PROVIDER_DIST_H) $(PROVIDER_NODIST_H)
PROVIDER_COMP_SRC   = $(PROVIDER_SRC) AutoTestInComp.cpp

CONSUMER_DIST_SRC   = AutoTestOut.cpp
CONSUMER_NODIST_SRC = $(IDL_SOURCES:.idl=Stub.cpp)
CONSUMER_SRC        = $(CONSUMER_DIST_SRC) $(CONSUMER_NODIST_SRC)

CONSUMER_DIST_H     = $(CONSUMER_DIST_SRC:.cpp=.h)
CONSUMER_NODIST_H   = $(CONSUMER_NODIST_SRC:.cpp=.h)
CONSUMER_H          = $(CONSUMER_DIST_H) $(CONSUMER_NODIST_H)
CONSUMER_COMP_SRC   = $(CONSUMER_SRC) AutoTestOutComp.cpp

#------------------------------------------------------------
# Object list
#------------------------------------------------------------
PROVIDER_OBJ=$(PROVIDER_DIST_SRC:.cpp=.o)
CONSUMER_OBJ=$(CONSUMER_DIST_SRC:.cpp=.o)

#------------------------------------------------------------
# Targets
#------------------------------------------------------------
# Targets directories
exampledir = $(rtm_exampledir)
socompdir  = $(exampledir)/rtc
sourcedir  = $(exampledir)/src/AutoTest

# Loadable RTCs
noinst_LTLIBRARIES = libAutoTestServiceSkel.la libAutoTestServiceStub.la
nodist_libAutoTestServiceSkel_la_SOURCES = $(PROVIDER_NODIST_SRC)
nodist_libAutoTestServiceStub_la_SOURCES = $(CONSUMER_NODIST_SRC)

socomp_LTLIBRARIES = AutoTestIn.la AutoTestOut.la

dist_AutoTestIn_la_SOURCES = $(PROVIDER_DIST_SRC) $(PROVIDER_DIST_H) 
AutoTestIn_la_LDFLAGS = -module
AutoTestIn_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.libs -L.
AutoTestIn_la_LIBADD  = -lAutoTestServiceSkel \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

dist_AutoTestOut_la_SOURCES = $(CONSUMER_DIST_SRC) $(CONSUMER_DIST_H)
AutoTestOut_la_LDFLAGS = -module
AutoTestOut_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.libs
AutoTestOut_la_LIBADD  = -lAutoTestServiceStub \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

# Standalone RTCs
example_PROGRAMS = AutoTestInComp AutoTestOutComp
AutoTestInComp_SOURCES = AutoTestInComp.cpp
AutoTestInComp_LDFLAGS = -L$(top_builddir)/src/lib/rtm/.libs
AutoTestInComp_LDADD   = .libs/AutoTestIn.a \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

AutoTestOutComp_SOURCES = AutoTestOutComp.cpp
AutoTestOutComp_LDFLAGS = -L$(top_builddir)/src/lib/rtm/.libs
AutoTestOutComp_LDADD   = .libs/AutoTestOut.a \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

# Example sources
source_DATA = $(ALL_SRC) $(EXTRA_DIST)

#------------------------------------------------------------
# Extra distributed files
#------------------------------------------------------------
EXTRA_DIST =                       \
	$(IDL_SOURCES)             \
	Makefile.AutoTestIn  \
	Makefile.AutoTestOut \
	README.AutoTestIn    \
	README.AutoTestOut   \
	rtc.conf

ALL_SRC =                    \
	$(IDL_SOURCES)       \
        $(PROVIDER_COMP_SRC) \
        $(CONSUMER_COMP_SRC)

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(ALL_SRC) > src.lst
	echo $(EXTRA_DIST) > other.lst

#------------------------------------------------------------
# Visual Studio Project
#------------------------------------------------------------
win32_builddir=$(top_builddir)/win32/OpenRTM-aist/examples/AutoTest

vcproj: vc8proj vc9proj vc10proj vc11proj vc12proj vc14proj

vc8proj: 
	$(top_builddir)/build/vcprojtool.py vcproj                   \
		--type EXE                                           \
		--vcversion "8.00"                                   \
		--projectname "AutoTestInComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestIn_vc8.vcproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(PROVIDER_COMP_SRC)                        \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestIn_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                      \
		--type DLL                                              \
		--vcversion "8.00"                                      \
		--projectname "AutoTestIn"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestInDll_vc8.vcproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(PROVIDER_SRC)                                \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestInDll_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                   \
		--type EXE                                           \
		--vcversion "8.00"                                   \
		--projectname "AutoTestOutComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestOut_vc8.vcproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                        \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOut_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                      \
		--type DLL                                              \
		--vcversion "8.00"                                      \
		--projectname "AutoTestOut"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestOutDll_vc8.vcproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                           \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOutDll_vc8.vcproj

vc9proj: 
	$(top_builddir)/build/vcprojtool.py vcproj                   \
		--type EXE                                           \
		--vcversion "9.00"                                   \
		--projectname "AutoTestInComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestIn_vc9.vcproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(PROVIDER_COMP_SRC)                        \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestIn_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                      \
		--type DLL                                              \
		--vcversion "9.00"                                      \
		--projectname "AutoTestIn"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestInDll_vc9.vcproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(PROVIDER_SRC)                                \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestInDll_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                   \
		--type EXE                                           \
		--vcversion "9.00"                                   \
		--projectname "AutoTestOutComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestOut_vc9.vcproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                        \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOut_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                      \
		--type DLL                                              \
		--vcversion "9.00"                                      \
		--projectname "AutoTestOut"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestOutDll_vc9.vcproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                           \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOutDll_vc9.vcproj

vc10proj: 
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "10.00"                                   \
		--projectname "AutoTestInComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestIn_vc10.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(PROVIDER_COMP_SRC)                        \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestIn_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "10.00"                                      \
		--projectname "AutoTestIn"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestInDll_vc10.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(PROVIDER_SRC)                                \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestInDll_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "10.00"                                   \
		--projectname "AutoTestOutComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestOut_vc10.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                        \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOut_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "10.00"                                      \
		--projectname "AutoTestOut"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestOutDll_vc10.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                           \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOutDll_vc10.vcxproj

vc11proj: 
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "11.00"                                   \
		--projectname "AutoTestInComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestIn_vc11.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(PROVIDER_COMP_SRC)                        \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestIn_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "11.00"                                      \
		--projectname "AutoTestIn"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestInDll_vc11.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(PROVIDER_SRC)                                \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestInDll_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "11.00"                                   \
		--projectname "AutoTestOutComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestOut_vc11.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                        \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOut_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "11.00"                                      \
		--projectname "AutoTestOut"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestOutDll_vc11.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                           \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOutDll_vc11.vcxproj

vc12proj: 
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "12.00"                                   \
		--projectname "AutoTestInComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestIn_vc12.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(PROVIDER_COMP_SRC)                        \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestIn_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "12.00"                                      \
		--projectname "AutoTestIn"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestInDll_vc12.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(PROVIDER_SRC)                                \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestInDll_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "12.00"                                   \
		--projectname "AutoTestOutComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestOut_vc12.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                        \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOut_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "12.00"                                      \
		--projectname "AutoTestOut"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestOutDll_vc12.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                           \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOutDll_vc12.vcxproj

vc14proj: 
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "14.00"                                   \
		--projectname "AutoTestInComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestIn_vc14.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(PROVIDER_COMP_SRC)                        \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestIn_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "14.00"                                      \
		--projectname "AutoTestIn"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestInDll_vc14.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(PROVIDER_SRC)                                \
		--header $(PROVIDER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestInDll_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type EXE                                           \
		--vcversion "14.00"                                   \
		--projectname "AutoTestOutComp"                \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/AutoTestOut_vc14.vcxproj \
		--yaml ../rtc.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                        \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOut_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                      \
		--type DLL                                              \
		--vcversion "14.00"                                      \
		--projectname "AutoTestOut"                       \
		--version $(RTM_VERSION)                                \
		--out $(win32_builddir)/AutoTestOutDll_vc14.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                            \
		--source $(CONSUMER_COMP_SRC)                           \
		--header $(CONSUMER_H)
	qkc -O- -sm $(win32_builddir)/AutoTestOutDll_vc14.vcxproj
	
dist-hook:


clean_objs:
	rm -f $(OBJS)

clean_skelstub:
	rm -f *Skel.h *Skel.cpp
	rm -f *Stub.h *Stub.cpp
	rm -f *.hh *SK.cc
	rm -f *C.cpp *C.h *S.cpp *S.h *_T.cpp *_T.h *.inl

clean-local: clean_objs clean_skelstub
	rm -f \
	*.bak *.rpo *.sym lib*.*_pure_* *.lst \
	Makefile.old *core *~ *loT
