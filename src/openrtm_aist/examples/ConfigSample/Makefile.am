## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for ConsoleIn/ConsoleOut components
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

#------------------------------------------------------------
# Targets
#------------------------------------------------------------
# Targets directories
exampledir = $(rtm_exampledir)
socompdir  = $(exampledir)/rtc
sourcedir  = $(exampledir)/src/ConfigSample

# Loadable RTCs
socomp_LTLIBRARIES = ConfigSample.la

ConfigSample_la_SOURCES = ConfigSample.cpp ConfigSample.h
ConfigSample_la_LDFLAGS = -module
ConfigSample_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.lib
ConfigSample_la_LIBADD  = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la


# Standalone RTCs
example_PROGRAMS = ConfigSampleComp
example_DATA     = configsample.conf

ConfigSampleComp_SOURCES = ConfigSampleComp.cpp 
ConfigSampleComp_DEPENDENCIES = ConfigSample.la
ConfigSampleComp_LDFLAGS = -L$(top_builddir)/src/lib/rtm/.libs
ConfigSampleComp_LDADD   = ConfigSample.o \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

# Example sources
source_DATA = $(ALL_SRC) $(EXTRA_DIST)

#------------------------------------------------------------
# Extra distributed files
#------------------------------------------------------------
EXTRA_DIST =                  \
	gen.sh                \
	rtc.conf              \
	configsample.conf     \
	VectorConvert.h       \
	Makefile.ConfigSample \
	README.ConfigSample

ALL_SRC =                    \
	ConfigSampleComp.cpp \
	ConfigSample.cpp     \
	ConfigSample.h

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(ALL_SRC) > src.lst
	echo $(EXTRA_DIST) > other.lst

#------------------------------------------------------------
# Visual Studio Project
#------------------------------------------------------------
win32_builddir=$(top_builddir)/win32/OpenRTM-aist/examples/ConfigSample/

vcproj: vc8proj vc9proj vc10proj vc11proj vc12proj vc14proj

vc8proj:
	$(top_builddir)/build/vcprojtool.py vcproj              \
		--type EXE                                      \
		--vcversion "8.00"                              \
		--projectname "ConfigSampleComp"                \
		--version $(RTM_VERSION)                        \
		--out $(win32_builddir)/ConfigSample_vc8.vcproj \
		--yaml ../rtc.vcproj.yaml                       \
		--source ConfigSample.cpp ConfigSampleComp.cpp  \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSample_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                 \
		--type DLL                                         \
		--vcversion "8.00"                                 \
		--projectname "ConfigSample"                       \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/ConfigSampleDll_vc8.vcproj \
		--yaml ../rtcdll.vcproj.yaml                       \
		--source ConfigSample.cpp                          \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSampleDll_vc8.vcproj

vc9proj:
	$(top_builddir)/build/vcprojtool.py vcproj              \
		--type EXE                                      \
		--vcversion "9.00"                              \
		--projectname "ConfigSampleComp"                \
		--version $(RTM_VERSION)                        \
		--out $(win32_builddir)/ConfigSample_vc9.vcproj \
		--yaml ../rtc.vcproj.yaml                       \
		--source ConfigSample.cpp ConfigSampleComp.cpp  \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSample_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj                 \
		--type DLL                                         \
		--vcversion "9.00"                                 \
		--projectname "ConfigSample"                       \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/ConfigSampleDll_vc9.vcproj \
		--yaml ../rtcdll.vcproj.yaml                       \
		--source ConfigSample.cpp                          \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSampleDll_vc9.vcproj

vc10proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj              \
		--type EXE                                      \
		--vcversion "10.00"                              \
		--projectname "ConfigSampleComp"                \
		--version $(RTM_VERSION)                        \
		--out $(win32_builddir)/ConfigSample_vc10.vcxproj \
		--yaml ../rtc.vcproj.yaml                       \
		--source ConfigSample.cpp ConfigSampleComp.cpp  \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSample_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                 \
		--type DLL                                         \
		--vcversion "10.00"                                 \
		--projectname "ConfigSample"                       \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/ConfigSampleDll_vc10.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                       \
		--source ConfigSample.cpp                          \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSampleDll_vc10.vcxproj

vc11proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj              \
		--type EXE                                      \
		--vcversion "11.00"                              \
		--projectname "ConfigSampleComp"                \
		--version $(RTM_VERSION)                        \
		--out $(win32_builddir)/ConfigSample_vc11.vcxproj \
		--yaml ../rtc.vcproj.yaml                       \
		--source ConfigSample.cpp ConfigSampleComp.cpp  \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSample_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                 \
		--type DLL                                         \
		--vcversion "11.00"                                 \
		--projectname "ConfigSample"                       \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/ConfigSampleDll_vc11.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                       \
		--source ConfigSample.cpp                          \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSampleDll_vc11.vcxproj

vc12proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj              \
		--type EXE                                      \
		--vcversion "12.00"                              \
		--projectname "ConfigSampleComp"                \
		--version $(RTM_VERSION)                        \
		--out $(win32_builddir)/ConfigSample_vc12.vcxproj \
		--yaml ../rtc.vcproj.yaml                       \
		--source ConfigSample.cpp ConfigSampleComp.cpp  \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSample_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                 \
		--type DLL                                         \
		--vcversion "12.00"                                 \
		--projectname "ConfigSample"                       \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/ConfigSampleDll_vc12.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                       \
		--source ConfigSample.cpp                          \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSampleDll_vc12.vcxproj

vc14proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj              \
		--type EXE                                      \
		--vcversion "14.00"                              \
		--projectname "ConfigSampleComp"                \
		--version $(RTM_VERSION)                        \
		--out $(win32_builddir)/ConfigSample_vc14.vcxproj \
		--yaml ../rtc.vcproj.yaml                       \
		--source ConfigSample.cpp ConfigSampleComp.cpp  \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSample_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj                 \
		--type DLL                                         \
		--vcversion "14.00"                                 \
		--projectname "ConfigSample"                       \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/ConfigSampleDll_vc14.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                       \
		--source ConfigSample.cpp                          \
		--header ConfigSample.h VectorConvert.h
	qkc -O- -sm $(win32_builddir)/ConfigSampleDll_vc14.vcxproj
	
dist-hook: lst vcproj


clean-local:
	rm -f *.bak *.rpo *.sym lib*.*_pure_* 
	rm -f *.lst *.core *~ *.so *.log
	rm -f Makefile.old 

