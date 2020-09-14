## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile for rtcd
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

AM_CPPFLAGS=-I$(top_srcdir) -I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_buiddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

bin_PROGRAMS = rtcd

rtcd_SOURCES = rtcd.cpp
rtcd_LDADD   = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
rtcd_LDFLAGS = -L$(top_builddir)/src/lib/rtm

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(RTC_SRC) > src.lst
	echo $(rtmheader_HEADERS) > header.lst

#------------------------------------------------------------
# Visual Studio Project
#------------------------------------------------------------
win32_builddir = $(top_builddir)/win32/OpenRTM-aist/

vcproj: vc8proj vc9proj vc10proj vc11proj vc12proj vc14proj

vc8proj: rtcd.yaml
	$(top_builddir)/build/vcprojtool.py vcproj             \
		--type RTCEXE                                      \
		--vcversion "8.00"                                 \
		--projectname "rtcd"                               \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/utils/rtcd/rtcd_vc8.vcproj \
		--yaml rtcd.yaml                                   \
		--source $(rtcd_SOURCES)
#	qkc -O- -sm $(win32_builddir)/rtm/libRTC_vc8.vcproj

vc9proj: rtcd.yaml
	$(top_builddir)/build/vcprojtool.py vcproj             \
		--type RTCEXE                                      \
		--vcversion "9.00"                                 \
		--projectname "rtcd"                               \
		--version $(RTM_VERSION)                           \
		--out $(win32_builddir)/utils/rtcd/rtcd_vc9.vcproj \
		--yaml rtcd.yaml                                   \
		--source $(rtcd_SOURCES)
#	qkc -O- -sm $(win32_builddir)/rtm/rtcd_vc9.vcproj

vc10proj: rtcd.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj             \
		--type RTCEXE                                        \
		--vcversion "10.00"                                  \
		--projectname "rtcd"                                 \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/utils/rtcd/rtcd_vc10.vcxproj \
		--yaml rtcd.yaml                                     \
		--source $(rtcd_SOURCES)

vc11proj: rtcd.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj             \
		--type RTCEXE                                        \
		--vcversion "11.00"                                  \
		--projectname "rtcd"                                 \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/utils/rtcd/rtcd_vc11.vcxproj \
		--yaml rtcd.yaml                                     \
		--source $(rtcd_SOURCES)

vc12proj: rtcd.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj             \
		--type RTCEXE                                        \
		--vcversion "12.00"                                  \
		--projectname "rtcd"                                 \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/utils/rtcd/rtcd_vc12.vcxproj \
		--yaml rtcd.yaml                                     \
		--source $(rtcd_SOURCES)
		
vc14proj: rtcd.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj             \
		--type RTCEXE                                        \
		--vcversion "14.00"                                  \
		--projectname "rtcd"                                 \
		--version $(RTM_VERSION)                             \
		--out $(win32_builddir)/utils/rtcd/rtcd_vc14.vcxproj \
		--yaml rtcd.yaml                                     \
		--source $(rtcd_SOURCES)
				
#------------------------------------------------------------
# for wxs file
#------------------------------------------------------------
wxs:
	$(top_srcdir)/build/makewxs.py flist \
		-c Rtcd \
		-o $(win32_builddir)installer/rtcd.yaml \
		-p ..\\bin \
		$(bin_PROGRAMS).exe

dist-hook: lst vcproj wxs

clean-local:
	rm -rf rtcd
	rm -rf *.o *~ *.core *.lst *.log
	rm -rf .libs 

EXTRA_DIST = rtcd.yaml

