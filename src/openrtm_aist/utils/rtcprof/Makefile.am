## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile for rtcd
##
## $Id: Makefile.am 1662 2010-01-15 10:45:51Z geoff $
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

AM_CPPFLAGS=-I$(top_srcdir) -I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_buiddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

bin_PROGRAMS = rtcprof

rtcprof_SOURCES = rtcprof.cpp
rtcprof_LDADD   = $(top_builddir)/src/lib/rtm/libRTC.la \
	$(top_builddir)/src/lib/coil/lib/libcoil.la
rtcprof_LDFLAGS = -L$(top_builddir)/src/lib/rtm

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

vc8proj: rtcprof.yaml
	$(top_builddir)/build/vcprojtool.py vcproj                   \
		--type RTCEXE                                            \
		--vcversion "8.00"                                       \
		--projectname "rtcprof"                                  \
		--version $(RTM_VERSION)                                 \
		--out $(win32_builddir)/utils/rtcprof/rtcprof_vc8.vcproj \
		--yaml rtcprof.yaml                                      \
		--source $(rtcprof_SOURCES)
#	qkc -O- -sm $(win32_builddir)/rtm/libRTC_vc8.vcproj

vc9proj: rtcprof.yaml
	$(top_builddir)/build/vcprojtool.py vcproj                   \
		--type RTCEXE                                            \
		--vcversion "9.00"                                       \
		--projectname "rtcprof"                                  \
		--version $(RTM_VERSION)                                 \
		--out $(win32_builddir)/utils/rtcprof/rtcprof_vc9.vcproj \
		--yaml rtcprof.yaml                                      \
		--source $(rtcprof_SOURCES)
#	qkc -O- -sm $(win32_builddir)/rtm/rtcprof_vc9.vcproj

vc10proj: rtcprof.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type RTCEXE                                              \
		--vcversion "10.00"                                        \
		--projectname "rtcprof"                                    \
		--version $(RTM_VERSION)                                   \
		--out $(win32_builddir)/utils/rtcprof/rtcprof_vc10.vcxproj \
		--yaml rtcprof.yaml                                        \
		--source $(rtcprof_SOURCES)

vc11proj: rtcprof.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type RTCEXE                                              \
		--vcversion "11.00"                                        \
		--projectname "rtcprof"                                    \
		--version $(RTM_VERSION)                                   \
		--out $(win32_builddir)/utils/rtcprof/rtcprof_vc11.vcxproj \
		--yaml rtcprof.yaml                                        \
		--source $(rtcprof_SOURCES)

vc12proj: rtcprof.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type RTCEXE                                              \
		--vcversion "12.00"                                        \
		--projectname "rtcprof"                                    \
		--version $(RTM_VERSION)                                   \
		--out $(win32_builddir)/utils/rtcprof/rtcprof_vc12.vcxproj \
		--yaml rtcprof.yaml                                        \
		--source $(rtcprof_SOURCES)
		
vc14proj: rtcprof.yaml
	$(top_builddir)/build/vcxprojtool.py vcxproj                   \
		--type RTCEXE                                              \
		--vcversion "14.00"                                        \
		--projectname "rtcprof"                                    \
		--version $(RTM_VERSION)                                   \
		--out $(win32_builddir)/utils/rtcprof/rtcprof_vc14.vcxproj \
		--yaml rtcprof.yaml                                        \
		--source $(rtcprof_SOURCES)
				
#------------------------------------------------------------
# for wxs file
#------------------------------------------------------------
wxs:
	$(top_srcdir)/build/makewxs.py flist \
		-c Rtcprof \
		-o $(win32_builddir)installer/rtcprof.yaml \
		-p ..\\bin \
		$(bin_PROGRAMS).exe

dist-hook: lst vcproj wxs

clean-local:
	rm -rf rtcprof *.o *~ *core .libs *.lst

EXTRA_DIST = rtcprof.yaml

