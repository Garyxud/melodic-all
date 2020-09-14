## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for deb package
##
##
## rpmpkg_build.sh.in   : RPM package build script template
##                        Only __VERSION__ is undefined.
## rpmpkg_build.sh      : RPM package build script created from 
##                        rpmpkg_build.sh.in replacing __VERSION__.
## OpenRTM-aist.spec.in : RPM spec file template
##
## OpenRTM-aist.spec    : RPM spec file created from OpenRTM-aist.spec
##
## $Id$
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

dist_key=`sh dist_key.sh`

all-local: OpenRTM-aist.spec rpm_build.sh
	sh rpm_build.sh

OpenRTM-aist.spec: OpenRTM-aist.spec.in
	echo "%define version         $(RTM_VERSION)" > $@
	echo "%define shortversion    $(RTM_MAJOR_VERSION).$(RTM_MINOR_VERSION)" >> $@
	echo "%define distname        $(dist_key)" >> $@
	echo "%define rtm_includedir  $(rtm_includedir)" >> $@
	echo "%define coil_includedir $(coil_includedir)" >> $@
	echo "%define rtm_datadir     $(rtm_datadir)" >> $@
	cat OpenRTM-aist.spec.in >> $@

rpm_build.sh: rpm_build.sh.in
	echo "#!/bin/sh" > $@
	echo "" >> $@
	echo "version=$(RTM_VERSION)" >> $@
	echo "" >> $@
	cat rpm_build.sh.in >> $@

clean-local:
	rm -f *~
	rm -f files
	rm -f OpenRTM-aist.spec rpm_build.sh
	rm -rf BUILD RPMS SOURCES SPECS SRPMS
	rm -f OpenRTM-aist-* 

EXTRA_DIST = \
	OpenRTM-aist.spec.in      \
	rpm_build.sh.in           \
	dist_key.sh               \
	fedora_repo.sh            \
	fedora_rpmcp.sh           \
	pkgbuild.sh               \
	vine_repo.sh              \
	vine_rpmcp.sh
