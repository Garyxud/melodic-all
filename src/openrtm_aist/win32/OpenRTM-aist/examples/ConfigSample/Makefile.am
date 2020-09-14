## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/examples/ConfigSample
##
## $Id$
##---------------------------------------------------------------------------
clean-local:
	rm -rf Debug Release
	rm -rf *~ *.cpp *.h *.idl *.conf
	rm -rf *.vcproj *.vcxproj *.user *.ncb *.suo
	rm -rf ConfigSample ConfigSampleComp

src:
	cp -p $(top_srcdir)/examples/ConfigSample/*.cpp .
	cp -p $(top_srcdir)/examples/ConfigSample/*.h .
	cp -p $(top_srcdir)/examples/ConfigSample/*.conf .
	qkc -O- -ms *.cpp
	qkc -O- -ms *.h
	qkc -O- -ms *.conf

dist-hook: src
