## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/rtm/idl
##
## $Id: Makefile.am 2711 2016-03-31 09:50:08Z kawauchi $
##---------------------------------------------------------------------------
clean-local:
	rm -rf Debug Release
	rm -f *~ *.idl *.cpp *.h *.cc *.hh
	rm -f *.sln *.suo *.ncb *.user *.vcproj *.vcxproj

src:
	cp -p $(top_srcdir)/src/lib/rtm/ext/*.idl .
	qkc -O- -ms *.idl


dist-hook: src
