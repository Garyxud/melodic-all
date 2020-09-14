## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/rtm
##
## $Id$
##---------------------------------------------------------------------------
SUBDIRS = idl . ext

RTM_VERSION = @RTM_VERSION@
RTM_SHORT_VERSION = @RTM_SHORT_VERSION@

clean-local:
	rm -rf Debug Release
	rm -f *~ *.cpp *.h *.cc *.hh version.txt
	rm -f *.sln *.suo *.ncb *.user *.vcproj *.vcxproj *.def

src:
	cp -p $(top_srcdir)/src/lib/rtm/*.cpp .
	cp -p $(top_srcdir)/src/lib/rtm/*.h .
	cp -p $(top_srcdir)/src/lib/rtm/version.txt .
	cp -p config_rtc.h.in config_rtc.h
	qkc -O- -ms *.cpp
	qkc -O- -ms *.h

dist-hook: src

EXTRA_DIST = config_rtc.h.in
