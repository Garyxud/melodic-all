## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist
##
## $Id$
##---------------------------------------------------------------------------
SUBDIRS = bin etc cmake build docs examples ext utils coil rtm installer .


clean-local:
	rm -f *~ *.sln *.yaml *.ncb *.suo *.user
	rm -f AUTHORS COPYING.LIB COPYRIGHT ChangeLog README README.jp
	rm -rf components
	rm -f coil_distbuild.vsprops
	rm -f coil_config.vsprops
	rm -f coil_distbuild.props
	rm -f coil_config.props
	rm -f build.bat

sln: OpenRTM-aist_vc8.sln OpenRTM-aist_vc9.sln OpenRTM-aist_vc10.sln OpenRTM-aist_vc11.sln OpenRTM-aist_vc12.sln OpenRTM-aist_vc14.sln

OpenRTM-aist_vc8.sln: OpenRTM-aist.sln.dep
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC8            \
		--dep OpenRTM-aist.sln.dep \
		--out OpenRTM-aist_vc8.sln \
		`find ./ -name '*_vc8.vcproj' \! -name 'USB*.vcproj' \! -name 'LTT*.vcproj' \! -name 'LogicalTime*.vcproj' \! -name 'FileName*.vcproj'`
	qkc -O- -ms OpenRTM-aist_vc8.sln

OpenRTM-aist_vc9.sln:
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC9            \
		--dep OpenRTM-aist.sln.dep \
		--out OpenRTM-aist_vc9.sln \
		`find ./ -name '*_vc9.vcproj' \! -name 'USB*.vcproj' \! -name 'LTT*.vcproj' \! -name 'LogicalTime*.vcproj' \! -name 'FileName*.vcproj'`
	qkc -O- -ms OpenRTM-aist_vc9.sln

OpenRTM-aist_vc10.sln:
	$(top_srcdir)/build/slntool.py      \
		--vcversion VC10            \
		--dep OpenRTM-aist.sln.dep  \
		--out OpenRTM-aist_vc10.sln \
		`find ./ -name '*_vc10.vcxproj' \! -name 'USB*.vcxproj' \! -name 'LTT*.vcxproj' \! -name 'LogicalTime*.vcxproj' \! -name 'FileName*.vcxproj'`
	qkc -O- -ms OpenRTM-aist_vc10.sln

OpenRTM-aist_vc11.sln:
	$(top_srcdir)/build/slntool.py      \
		--vcversion VC11            \
		--dep OpenRTM-aist.sln.dep  \
		--out OpenRTM-aist_vc11.sln \
		`find ./ -name '*_vc11.vcxproj' \! -name 'USB*.vcxproj' \! -name 'LTT*.vcxproj' \! -name 'LogicalTime*.vcxproj' \! -name 'FileName*.vcxproj'`
	qkc -O- -ms OpenRTM-aist_vc11.sln

OpenRTM-aist_vc12.sln:
	$(top_srcdir)/build/slntool.py      \
		--vcversion VC12            \
		--dep OpenRTM-aist.sln.dep  \
		--out OpenRTM-aist_vc12.sln \
		`find ./ -name '*_vc12.vcxproj' \! -name 'USB*.vcxproj' \! -name 'LTT*.vcxproj' \! -name 'LogicalTime*.vcxproj' \! -name 'FileName*.vcxproj'`
	qkc -O- -ms OpenRTM-aist_vc12.sln
	
OpenRTM-aist_vc14.sln:
	$(top_srcdir)/build/slntool.py      \
		--vcversion VC14            \
		--dep OpenRTM-aist.sln.dep  \
		--out OpenRTM-aist_vc14.sln \
		`find ./ -name '*_vc14.vcxproj' \! -name 'USB*.vcxproj' \! -name 'LTT*.vcxproj' \! -name 'LogicalTime*.vcxproj' \! -name 'FileName*.vcxproj'`
	qkc -O- -ms OpenRTM-aist_vc14.sln

vsprops:  coil_distbuild.vsprops coil_config.vsprops coil_distbuild.props coil_config.props


coil_distbuild.vsprops:
	cp -p $(top_srcdir)/src/lib/coil/win32/coil_distbuild.vsprops .

coil_config.vsprops:
	cp -p $(top_srcdir)/src/lib/coil/win32/coil_config.vsprops .

coil_distbuild.props:
	cp -p $(top_srcdir)/src/lib/coil/win32/coil_distbuild.props .

coil_config.props:
	cp -p $(top_srcdir)/src/lib/coil/win32/coil_config.props .

build.bat:
	sed -e 's/__VERSION__/$(VERSION)/g' build.bat.in > build.bat	

AUTHORS:
	cp $(top_srcdir)/AUTHORS     .
ChangeLog:
	cp $(top_srcdir)/ChangeLog   .
COPYING.LIB:
	cp $(top_srcdir)/COPYING.LIB .
COPYRIGHT:
	cp $(top_srcdir)/COPYRIGHT   .
README:
	cp $(top_srcdir)/README      .
README.jp:
	cp $(top_srcdir)/README.jp   .


dist-hook: clean-local sln vsprops build.bat

EXTRA_DIST =                      \
	OpenRTM-aist.sln.dep          \
	autobuild_vc8.bat             \
	autobuild_vc9.bat             \
	autobuild_vc10.bat            \
	coil_distbuild.vsprops        \
	coil_config.vsprops           \
	rtm_distbuild.vsprops         \
	rtm_config.vsprops            \
	coil_distbuild.props          \
	coil_config.props             \
	rtm_distbuild.props           \
	rtm_config.props              \
	make_package.sh               \
	prepare_openrtm.sh            \
	prepare_openrtm_inst.sh       \
	AUTHORS                       \
	ChangeLog                     \
	COPYING.LIB                   \
	COPYRIGHT                     \
	README                        \
	README.jp


