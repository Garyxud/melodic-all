## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile for rtm-config: rtm configuration information viewer
##
## $Id: Makefile.am 849 2008-09-25 11:20:51Z n-ando $
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

pkgconfigdir = @pkgconfigdir@
pkgconfig_DATA = libcoil.pc

bin_SCRIPTS = coil-config

coil-config: coil-config.in
	echo '#!/bin/sh' > $@
	echo '' >> $@
	echo 'coil_prefix="'${prefix}'"' >> $@
	echo 'coil_exec_prefix="'${exec_prefix}'"' >> $@
	echo 'coil_cxx="'${CXX}'"' >> $@
	echo 'coil_cflags="'${CXXFLAGS}' -I'${coil_includedir}'"' >> $@
	echo 'coil_libs="'${LDFLAGS}' -L'${prefix}'/lib '${LIBS}' -lRTC"' >> $@
	echo 'coil_libdir="'${coil_libdir}'"' >> $@
	echo 'coil_version="'${COIL_VERSION}'"' >> $@
	cat coil-config.in >> $@
	chmod 755 $@

EXTRA_DIST = coil-config.in

clean-local:
	rm -f libcoil.pc
	rm -f coil-config
	rm -rf *~ *core
