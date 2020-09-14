## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/installer
##
## $Id$
##---------------------------------------------------------------------------
clean-local:
	rm -f autowix.cmd
	rm -f OpenRTM-aist.wxs.yaml
	rm -f OpenRTM-aist.wxs
	rm -f *.yaml *~ autowix.cmd
	rm -f *.wixobj *.wixpdb *.mst *.msi 

autowix.cmd:
	sed -e 's/__VERSION__/$(VERSION)/g' autowix.cmd.in > autowix.cmd

OpenRTM-aist.wxs.yaml:
	sed -e 's/__VERSION__/$(VERSION)/g' OpenRTM-aist.wxs.yaml.in > OpenRTM-aist.wxs.yaml.1
	sed -e 's/__SHORT_VERSION__/$(RTM_MAJOR_VERSION)\.$(RTM_MINOR_VERSION)/g' OpenRTM-aist.wxs.yaml.1 > OpenRTM-aist.wxs.yaml
	rm -f OpenRTM-aist.wxs.yaml.1

OpenRTM-aist.wxs:
	$(top_srcdir)/build/makewxs.py wxs \
		-o OpenRTM-aist.wxs \
		-i OpenRTM-aist.wxs.in \
		*.yaml

dist-hook: autowix.cmd OpenRTM-aist.wxs.yaml OpenRTM-aist.wxs

EXTRA_DIST =                       \
	WiLangId.vbs               \
	WiSubStg.vbs               \
	OpenRTM-aist.wxs.in        \
	OpenRTM-aist.wxs.yaml.in   \
	autowix.cmd.in             \
	License.rtf                \
	langs.txt                  \
	crean.cmd                  \
	omniORB/omniORB_inc.wxs.in \
	omniORB/omniwxs.py         \
	OpenRTP/OpenRTP_inc.wxs.in \
	OpenRTP/OpenRTP_inc.wxs    \
	OpenRTP/openrtpwxs.py      \
	OpenCV2.1/OpenCV_inc.wxs.in \
	OpenCV2.1/opencvwxs.py \
	OpenCV-RTC/OpenCV-RTC_inc.wxs.in \
	OpenCV-RTC/opencvrtcwxs.py

DIST_SUBDIRS = Bitmaps
