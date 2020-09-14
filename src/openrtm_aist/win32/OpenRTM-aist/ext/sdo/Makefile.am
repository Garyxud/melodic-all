## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/ext/sdo
##
## $Id: Makefile.am 1680 2010-01-17 15:12:59Z n-ando $
##---------------------------------------------------------------------------
SUBDIRS = observer .

wxs:
	cp -p sdo.yaml ../../installer/

clean-local:
	rm -f *~

dist-hook: clean-local wxs

EXTRA_DIST = sdo.yaml
