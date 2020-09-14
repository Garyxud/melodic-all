#!/bin/sh
#
# @file platform.sh
# @brief script for settingup coil platform files
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

#
# platform.sh [--set|--unset] platform
#
# --set  : make symbolic link of coil platform dependent files
# --unset: remove symbolick link of coil platform dependent files
#


set_files () {
    platform=$1
    if test ! -d $platform; then
	echo "No such platform:" $platform
	exit 1
    fi

    files=`find ${platform} -name "*_${platform}.h" -or -name "*_${platform}.cpp"`
    for f in $files; do
	fname=`echo $f | sed -e "s/^$ostype\///" | sed -e "s/_$ostype//"`
	if test -L $fname; then
	    rm $fname
	fi
	if test -f $fname; then
	    echo "File already exists:" $fname
	    echo "abort"
	    exit 1
	fi
	ln -s $f $fname
    done
}

unset_files () {
    platform=$1
    if test ! -d $platform; then
	echo "No such platform: " $platform
	exit 1
    fi

    files=`find ${platform} -name "*_${platform}.h" -or -name "*_${platform}.cpp"`
    echo $files
    for f in $files; do
	fname=`echo $f | sed -e "s/^$ostype\///" | sed -e "s/_$ostype//"`
	if test -L $fname; then
	    echo "remove:" $fname
	    rm $fname
	fi
    done
}

usage () {
    cat <<EOF
usage: $0 [--set|--unset] platform

example:
  set coil files to build directory
  > $0 -set posix
  unset(rmove) coil fils from build directory
  > $0 --unset posix
EOF
}


if test $# != 2; then
    usage
    exit 1
fi

cmd=$1
ostype=`echo $2 | tr "[A-Z]" "[a-z]"`

case $cmd in
    -set)
	set_files $ostype
	;;
    -unset)
	unset_files $ostype
	;;
    *)
	usage
	exit 1
esac
