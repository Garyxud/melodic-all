## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for the top directory
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

SUBDIRS = build src utils examples etc $(DOCS) win32

DIST_SUBDIRS = packages $(SUBDIRS)

EXTRA_DIST =        \
	README      \
	README.jp   \
	COPYRIGHT   \
	COPYING.LIB \
	INSTALL     \
	INSTALL.jp

clean-local:
	rm -f *~ *.core
	rm -f build-arch-stamp build-indep-stamp
	rm -f compile
	rm -rf debian
	rm -f OpenRTM-aist*.tar.gz
	rm -f OpenRTM-aist*.tar.bz2
	rm -f OpenRTM-aist*.zip

