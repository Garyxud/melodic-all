## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for coil (Component Object Infrastructure Layer)
##
## $Id$
##---------------------------------------------------------------------------

if COIL_PLATFORM_ACE
PLATFORM = ace
NO_PLATFORM = posix
endif
if COIL_PLATFORM_POSIX
PLATFORM = posix
NO_PLATFORM = ace
endif

SUBDIRS = include $(PLATFORM) lib build common utils @UNITTEST@
DIST_SUBDIRS = $(SUBDIRS) $(NO_PLATFORM) win32 tests

clean-local:
	rm -f config_coil.h
	rm -f *~
	rm -f compile
	rm -rf coil-$(VERSION)
	rm -f coil-$(VERSION).tar.gz
	rm -f coil-$(VERSION).tar.bz2
