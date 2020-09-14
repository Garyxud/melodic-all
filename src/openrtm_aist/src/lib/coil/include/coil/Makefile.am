## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for coil (Component Object Infrastructure Layer)
##
## $Id$
##---------------------------------------------------------------------------

if COIL_PLATFORM_ACE
PLATFORM = ace
endif
if COIL_PLATFORM_POSIX
PLATFORM = posix
endif

all-local:
	cp -p $(top_srcdir)/$(PLATFORM)/coil/*.h .
	cp -p $(top_srcdir)/common/*.h .

clean-local:
	mv config_coil.h config_coil_tmp
	rm -f *.h
	mv config_coil_tmp config_coil.h
	rm -f *~

dist-hook: all-local
