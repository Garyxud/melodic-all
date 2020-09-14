## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for coil (Component Object Infrastructure Layer)
##
## $Id$
##---------------------------------------------------------------------------

SUBDIRS = coil bin build .

clean-local:
	rm -f *~
	rm -f *.sln *.ncb *.suo

EXTRA_DIST =                   \
	coil                   \
	coil_ace.sln.dep       \
	coil_config.vsprops    \
	coil_distbuild.vsprops \
	coil_ace_vc8.sln       \
	coil_ace_vc9.sln

#------------------------------------------------------------
# sln file build rules
#------------------------------------------------------------
win32_builddir = .

sln: coil_ace_vc8.sln coil_ace_vc9.sln  coil_ace_vc10.sln

vcproj:
	(cd coil ; make vcproj)


coil_ace_vc8.sln: coil_ace.sln.dep vcproj
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC8            \
		--dep coil_ace.sln.dep \
		--out coil_ace_vc8.sln \
		`find ./ -name '*_vc8.vcproj'`
	qkc -O- -ms coil_ace_vc8.sln

coil_ace_vc9.sln: coil_ace.sln.dep vcproj
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC9            \
		--dep coil_ace.sln.dep \
		--out coil_ace_vc9.sln \
		`find ./ -name '*_vc9.vcproj'`
	qkc -O- -ms coil_ace_vc9.sln

coil_ace_vc10.sln: coil_ace.sln.dep vcproj
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC10            \
		--dep coil_ace.sln.dep \
		--out coil_ace_vc10.sln \
		`find ./ -name '*_vc10.vcproj'`
	qkc -O- -ms coil_ace_vc10.sln

