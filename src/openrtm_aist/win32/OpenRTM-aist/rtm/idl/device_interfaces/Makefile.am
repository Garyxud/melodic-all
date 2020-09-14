## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/rtm/idl/device_interfaces
##
## $Id: Makefile.am 2556 2014-04-09 17:05:15Z n-ando $
##---------------------------------------------------------------------------
clean-local:
	rm -rf Debug Release
	rm -f *~ *.idl *.cpp *.h *.cc *.hh
	rm -f *.sln *.suo *.ncb *.user *.vcproj *.vcxproj

src:
	cp -p $(top_srcdir)/src/lib/rtm/idl/device_interfaces/*.idl .
	qkc -O- -ms *.idl


dist-hook: src
