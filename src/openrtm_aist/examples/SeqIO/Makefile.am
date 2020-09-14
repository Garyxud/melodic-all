## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for SeqIn/SeqOut components
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
socompdir  = $(rtm_exampledir)/rtc
sourcedir  = $(rtm_exampledir)/src/SeqIO

# Loadable RTCs
socomp_LTLIBRARIES = SeqIn.la SeqOut.la

SeqIn_la_SOURCES = SeqIn.cpp SeqIn.h
SeqIn_la_LDFLAGS = -module
SeqIn_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.lib
SeqIn_la_LIBADD  = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

SeqOut_la_SOURCES = SeqOut.cpp SeqOut.h
SeqOut_la_LDFLAGS = -module
SeqOut_la_FLAGS   = -L$(top_builddir)/src/lib/rtm/.lib
SeqOut_la_LIBADD  = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

# Standalone RTCs
example_PROGRAMS = SeqInComp SeqOutComp
example_DATA = rtc.conf component.conf

SeqInComp_SOURCES = SeqInComp.cpp 
SeqInComp_DEPENDENCIES = SeqIn.la
SeqInComp_LDFLAGS = -L$(top_builddir)/src/lib/rtm/.libs
SeqInComp_LDADD = SeqIn.o \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

SeqOutComp_SOURCES = SeqOutComp.cpp 
SeqOutComp_DEPENDENCIES = SeqOut.la
SeqOutComp_LDFLAGS = -L$(top_builddir)/src/lib/rtm/.libs
SeqOutComp_LDADD =  SeqOut.o \
	$(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

# Example sources
source_DATA = $(ALL_SRC) $(EXTRA_DIST)

#------------------------------------------------------------
# Extra distribution files
#------------------------------------------------------------
EXTRA_DIST =            \
	Makefile.SeqIn  \
	Makefile.SeqOut \
	README.SeqIn    \
	README.SeqOut   \
	gen.sh          \
	rtc.conf        \
	component.conf

ALL_SRC =              \
	SeqInComp.cpp  \
	SeqIn.cpp      \
	SeqIn.h        \
	SeqOutComp.cpp \
	SeqOut.cpp     \
	SeqOut.h

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(ALL_SRC) > src.lst
	echo $(EXTRA_DIST) > other.lst

#------------------------------------------------------------
# Visual Studio Project
#------------------------------------------------------------
win32_builddir=$(top_builddir)/win32/OpenRTM-aist/examples/SeqIO

vcproj: vc8proj vc9proj vc10proj vc11proj vc12proj vc14proj

vc8proj: 
	$(top_builddir)/build/vcprojtool.py vcproj       \
		--type EXE                               \
		--vcversion "8.00"                       \
		--projectname "SeqInComp"                \
		--version $(RTM_VERSION)                 \
		--out $(win32_builddir)/SeqIn_vc8.vcproj \
		--yaml ../rtc.vcproj.yaml                \
		--source SeqIn.cpp SeqInComp.cpp         \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqIn_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj          \
		--type DLL                                  \
		--vcversion "8.00"                          \
		--projectname "SeqIn"                       \
		--version $(RTM_VERSION)                    \
		--out $(win32_builddir)/SeqInDll_vc8.vcproj \
		--yaml ../rtcdll.vcproj.yaml                \
		--source SeqIn.cpp                          \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqInDll_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj        \
		--type EXE                                \
		--vcversion "8.00"                        \
		--projectname "SeqOutComp"                \
		--version $(RTM_VERSION)                  \
		--out $(win32_builddir)/SeqOut_vc8.vcproj \
		--yaml ../rtc.vcproj.yaml                 \
		--source SeqOut.cpp SeqOutComp.cpp        \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOut_vc8.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj           \
		--type DLL                                   \
		--vcversion "8.00"                           \
		--projectname "SeqOut"                       \
		--version $(RTM_VERSION)                     \
		--out $(win32_builddir)/SeqOutDll_vc8.vcproj \
		--yaml ../rtcdll.vcproj.yaml                 \
		--source SeqOut.cpp                          \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOutDll_vc8.vcproj

vc9proj:
	$(top_builddir)/build/vcprojtool.py vcproj       \
		--type EXE                               \
		--vcversion "9.00"                       \
		--projectname "SeqInComp"                \
		--version $(RTM_VERSION)                 \
		--out $(win32_builddir)/SeqIn_vc9.vcproj \
		--yaml ../rtc.vcproj.yaml                \
		--source SeqIn.cpp SeqInComp.cpp         \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqIn_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj          \
		--type DLL                                  \
		--vcversion "9.00"                          \
		--projectname "SeqIn"                       \
		--version $(RTM_VERSION)                    \
		--out $(win32_builddir)/SeqInDll_vc9.vcproj \
		--yaml ../rtcdll.vcproj.yaml                \
		--source SeqIn.cpp                          \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqInDll_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj        \
		--type EXE                                \
		--vcversion "9.00"                        \
		--projectname "SeqOutComp"                \
		--version $(RTM_VERSION)                  \
		--out $(win32_builddir)/SeqOut_vc9.vcproj \
		--yaml ../rtc.vcproj.yaml                 \
		--source SeqOut.cpp SeqOutComp.cpp        \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOut_vc9.vcproj
	$(top_builddir)/build/vcprojtool.py vcproj           \
		--type DLL                                   \
		--vcversion "9.00"                           \
		--projectname "SeqOut"                       \
		--version $(RTM_VERSION)                     \
		--out $(win32_builddir)/SeqOutDll_vc9.vcproj \
		--yaml ../rtcdll.vcproj.yaml                 \
		--source SeqOut.cpp                          \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOutDll_vc9.vcproj

vc10proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj       \
		--type EXE                               \
		--vcversion "10.00"                       \
		--projectname "SeqInComp"                \
		--version $(RTM_VERSION)                 \
		--out $(win32_builddir)/SeqIn_vc10.vcxproj \
		--yaml ../rtc.vcproj.yaml                \
		--source SeqIn.cpp SeqInComp.cpp         \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqIn_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj          \
		--type DLL                                  \
		--vcversion "10.00"                          \
		--projectname "SeqIn"                       \
		--version $(RTM_VERSION)                    \
		--out $(win32_builddir)/SeqInDll_vc10.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                \
		--source SeqIn.cpp                          \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqInDll_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj        \
		--type EXE                                \
		--vcversion "10.00"                        \
		--projectname "SeqOutComp"                \
		--version $(RTM_VERSION)                  \
		--out $(win32_builddir)/SeqOut_vc10.vcxproj \
		--yaml ../rtc.vcproj.yaml                 \
		--source SeqOut.cpp SeqOutComp.cpp        \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOut_vc10.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj           \
		--type DLL                                   \
		--vcversion "10.00"                           \
		--projectname "SeqOut"                       \
		--version $(RTM_VERSION)                     \
		--out $(win32_builddir)/SeqOutDll_vc10.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                 \
		--source SeqOut.cpp                          \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOutDll_vc10.vcxproj

vc11proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj       \
		--type EXE                               \
		--vcversion "11.00"                       \
		--projectname "SeqInComp"                \
		--version $(RTM_VERSION)                 \
		--out $(win32_builddir)/SeqIn_vc11.vcxproj \
		--yaml ../rtc.vcproj.yaml                \
		--source SeqIn.cpp SeqInComp.cpp         \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqIn_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj          \
		--type DLL                                  \
		--vcversion "11.00"                          \
		--projectname "SeqIn"                       \
		--version $(RTM_VERSION)                    \
		--out $(win32_builddir)/SeqInDll_vc11.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                \
		--source SeqIn.cpp                          \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqInDll_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj        \
		--type EXE                                \
		--vcversion "11.00"                        \
		--projectname "SeqOutComp"                \
		--version $(RTM_VERSION)                  \
		--out $(win32_builddir)/SeqOut_vc11.vcxproj \
		--yaml ../rtc.vcproj.yaml                 \
		--source SeqOut.cpp SeqOutComp.cpp        \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOut_vc11.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj           \
		--type DLL                                   \
		--vcversion "11.00"                           \
		--projectname "SeqOut"                       \
		--version $(RTM_VERSION)                     \
		--out $(win32_builddir)/SeqOutDll_vc11.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                 \
		--source SeqOut.cpp                          \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOutDll_vc11.vcxproj

vc12proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj       \
		--type EXE                               \
		--vcversion "12.00"                       \
		--projectname "SeqInComp"                \
		--version $(RTM_VERSION)                 \
		--out $(win32_builddir)/SeqIn_vc12.vcxproj \
		--yaml ../rtc.vcproj.yaml                \
		--source SeqIn.cpp SeqInComp.cpp         \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqIn_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj          \
		--type DLL                                  \
		--vcversion "12.00"                          \
		--projectname "SeqIn"                       \
		--version $(RTM_VERSION)                    \
		--out $(win32_builddir)/SeqInDll_vc12.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                \
		--source SeqIn.cpp                          \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqInDll_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj        \
		--type EXE                                \
		--vcversion "12.00"                        \
		--projectname "SeqOutComp"                \
		--version $(RTM_VERSION)                  \
		--out $(win32_builddir)/SeqOut_vc12.vcxproj \
		--yaml ../rtc.vcproj.yaml                 \
		--source SeqOut.cpp SeqOutComp.cpp        \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOut_vc12.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj           \
		--type DLL                                   \
		--vcversion "12.00"                           \
		--projectname "SeqOut"                       \
		--version $(RTM_VERSION)                     \
		--out $(win32_builddir)/SeqOutDll_vc12.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                 \
		--source SeqOut.cpp                          \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOutDll_vc12.vcxproj

vc14proj:
	$(top_builddir)/build/vcxprojtool.py vcxproj       \
		--type EXE                               \
		--vcversion "14.00"                       \
		--projectname "SeqInComp"                \
		--version $(RTM_VERSION)                 \
		--out $(win32_builddir)/SeqIn_vc14.vcxproj \
		--yaml ../rtc.vcproj.yaml                \
		--source SeqIn.cpp SeqInComp.cpp         \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqIn_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj          \
		--type DLL                                  \
		--vcversion "14.00"                          \
		--projectname "SeqIn"                       \
		--version $(RTM_VERSION)                    \
		--out $(win32_builddir)/SeqInDll_vc14.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                \
		--source SeqIn.cpp                          \
		--header SeqIn.h
	qkc -O- -sm $(win32_builddir)/SeqInDll_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj        \
		--type EXE                                \
		--vcversion "14.00"                        \
		--projectname "SeqOutComp"                \
		--version $(RTM_VERSION)                  \
		--out $(win32_builddir)/SeqOut_vc14.vcxproj \
		--yaml ../rtc.vcproj.yaml                 \
		--source SeqOut.cpp SeqOutComp.cpp        \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOut_vc14.vcxproj
	$(top_builddir)/build/vcxprojtool.py vcxproj           \
		--type DLL                                   \
		--vcversion "14.00"                           \
		--projectname "SeqOut"                       \
		--version $(RTM_VERSION)                     \
		--out $(win32_builddir)/SeqOutDll_vc14.vcxproj \
		--yaml ../rtcdll.vcproj.yaml                 \
		--source SeqOut.cpp                          \
		--header SeqOut.h
	qkc -O- -sm $(win32_builddir)/SeqOutDll_vc14.vcxproj
	
dist-hook: lst vcproj

clean-local:
	rm -f *.bak *.rpo *.sym lib*.*_pure_* 
	rm -f *.lst *.core *~ *.so *.log
	rm -f Makefile.old 
