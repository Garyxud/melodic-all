#!/bin/sh
#
# @file pkg_install_debianSarge.sh
# @brief OpenRTM-aist dependent packages install script for Debian-sarge
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#

ace="ace-5.4"
ace_pkg="libace5.4"

ace_devel="ace-devel-5.4"
ace_devel_pkg="libace-dev"

boost_devel="boost-devel-1.32"
boost_devel_pkg="libboost-dev"

boost_regex_devel="boost-regex-devel-1.32"
boost_regex_devel_pkg="libboost-regex-dev"

omniorb="omniORB-4.0.6"
omniorb_pkg="libomniorb4"

omniorb_devel="omniORB-devel-4.0.6"
omniorb_devel_pkg="libomniorb4-dev"

omniidl="omniidl-4.0.6"
omniidl_pkg="omniidl4"

omninames="omniNames-4.0.6"
omninames_pkg="omniorb4-nameserver"

omniorbpy="py23-omniorb-2.6"
omniorbpy_pkg="python2.3-omniorb2-omg"

gcc="gcc-3.3"
gcc_pkg="gcc3.3"

make="make-3.8"
make_pkg="make"

gpp="g++-3.3"
gpp_pkg="g++"


pkg_install () {
    echo '------------------------------------------------------------'
    echo 'Installing' $1
    echo ''
    aptitude install $2
}

pkg_uninstall () {
    echo '------------------------------------------------------------'
    aptitude remove $2
}

pkg_install_all () {
    pkg_install $ace $ace_pkg
    pkg_install $ace_devel $ace_devel_pkg
    pkg_install $boost_devel $boost_devel_pkg
    pkg_install $boost_regex_devel $boost_regex_devel_pkg
    pkg_install $omniorb $omniorb_pkg
    pkg_install $omniorb_devel $omniorb_devel_pkg
    pkg_install $omniidl $omniidl_pkg
    pkg_install $omninames $omninames_pkg
    pkg_install $omniorbpy $omniorbpy_pkg
    pkg_install $gcc $gcc_pkg
    pkg_install $make $make_pkg
    pkg_install $gpp $gpp_pkg
}


pkg_uninstall_all () {
    pkg_uninstall $ace $ace_pkg
    pkg_uninstall $ace_devel $ace_devel_pkg
    pkg_uninstall $boost_devel $boost_devel_pkg
    pkg_uninstall $boost_regex_devel $boost_regex_devel_pkg
    pkg_uninstall $omniorb $omniorb_pkg
    pkg_uninstall $omniorb_devel $omniorb_devel_pkg
    pkg_uninstall $omniidl $omniidl_pkg
    pkg_uninstall $omninames $omninames_pkg
    pkg_uninstall $omniorbpy $omniorbpy_pkg
}

if test "x$1" = "x-u" ; then
    pkg_uninstall_all
else
    pkg_install_all
fi

