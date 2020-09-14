## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/bin
##
## $Id$
##---------------------------------------------------------------------------

VER = $(RTM_SHORT_VERSION)
WINLIBS = libRTCSkel.lib \
	libRTCSkeld.lib  \
	RTC$(VER).l\ib   \
	RTC$(VER)d.lib   \
	RTC$(VER).dll    \
	RTC$(VER)d.dll   \
	coil$(VER).lib   \
	coil$(VER)d.lib  \
	coil$(VER).dll   \
	coil$(VER)d.dll

clean-local:
	rm -f *~ *.dll *.lib *.exe
	rm -rf *~ *.py

copy_scripts:
	cp -p $(top_srcdir)/build/*.py .
	cp -p $(top_srcdir)/utils/rtm-skelwrapper/*.py .
	cp -p $(top_srcdir)/utils/rtm-skelwrapper/rtm-skelwrapper rtm-skelwrapper.py
wxs:
	$(top_srcdir)/build/makewxs.py flist \
		-c Libs \
		-o ../installer/libs.yaml \
		-p ..\\bin \
		$(WINLIBS)
	$(top_srcdir)/build/makewxs.py flist \
		-c Bin \
		-o ../installer/bin.yaml \
		-p ..\\bin \
		*.py *.bat rtc.conf $(WINLIBS)

dist-hook: clean-local copy_scripts wxs

EXTRA_DIST = \
	rtm-naming.bat \
	rtc.conf
