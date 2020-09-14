#!/bin/sh
#
# @file vine_repo
# @brief apt-rpm repository database creation for VineLinux
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

# Base directory of repository
basedir="/exports/pub/repository/pub/Linux/Vine/apt"
pkgdir="/usr/users/builder/PackageBuild/"
# VineLinux versions
versions="3.1 4.2 5.0"

# VineLinux architectures
archs="i386 SRPMS"

for version in $versions; do
    for arch in $archs ; do
	sver=`echo $version | sed 's/\.//g'`
	if test "x$arch" = "xSRPMS"; then
	    rpmdir=$basedir/$version/$arch/SRPMS.main
	    sarch="src"
	else
	    rpmdir=$basedir/$version/$arch/RPMS.main
	    sarch=$arch
	fi

	echo ""
	echo "Copying rpms to repository:"
	echo "dir:  "$rpmdir
	name="*.vl$sver.$sarch.rpm"
	name_noarch="*.vl$sver.noarch.rpm"
	rpms=`/usr/bin/find $pkgdir -name $name`
	rpms_noarch=`/usr/bin/find $pkgdir -name $name_noarch | grep i686`
	rpms="${rpms} ${rpms_noarch}"
	for rpm in $rpms ; do
	    rpmname=`basename $rpm`
	    if test -f $rpmdir/$rpmname ; then
		read -p "Overwrite? $rpmname [Y/n]" ow
		if test ! "$ow" = "n" ; then
		    cp $rpm $rpmdir
		    echo "copy done"
		fi
	    else
		cp $rpm $rpmdir
		echo "copy done"
	    fi
	done
    done
done
