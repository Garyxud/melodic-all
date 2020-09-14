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
basedir=/exports/pub/repository/pub/Linux/Fedora/releases
pkgdir="/usr/users/builder/PackageBuild/"

# Fedora releases
versions="4 5 6 7 8 9 10 11 12 13" 
versions="13" 

# Fedora architectures
archs="i386 x86_64 source"



for version in $versions; do
    for arch in $archs ; do
	if test "x$arch" = "xsource"; then
	    rpmdir=$basedir/$version/Fedora/$arch/SRPMS
	    sarch="src"
	else
#	    rpmdir=$basedir/$version/Fedora/$arch/os/Packages
	    rpmdir=$basedir/$version/Fedora/$arch/os/Packages/OpenRTM-aist-1.0
	    sarch=$arch
	    if test "x$arch" = "xi386"; then
		karch="i686"
		sarch="i*86"
	    else
		karch=$sarch
	    fi
	fi
	if ! test -d $rpmdir ; then
	    mkdir -p $rpmdir
	    echo "Directory created."
	    echo "  "$rpmdir
	fi
	echo ""
	echo "Copying rpms to repository:"
	echo "dir:  "$rpmdir
	name="*.fc$version.$sarch.rpm"
	echo "name: "$name
	rpms=`/usr/bin/find $pkgdir -name $name`
	rpms_noarch=""
	if test ! "x$arch" = "xsource"; then
	    name_noarch="*.fc$version.noarch.rpm"
	    rpms_noarch=`/usr/bin/find $pkgdir -name $name_noarch |grep $karch`
	fi
	rpms="$rpms $rpms_noarch"
	for rpm in $rpms ; do
	    rpmname=`basename $rpm`
	    echo $rpmname
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
