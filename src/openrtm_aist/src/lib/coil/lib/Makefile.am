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
	rm -rf .libs
	rm -f libcoil.la
	ln -s $(top_srcdir)/$(PLATFORM)/coil/.libs .
	ln -s $(top_srcdir)/$(PLATFORM)/coil/libcoil.la .

clean-local:
	rm -f *~
	rm -rf .libs
	rm -f libcoil*
