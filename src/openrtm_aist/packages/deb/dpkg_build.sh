#!/bin/sh 
#
# Debian package build script
#
# the following files are constant
# - README.Debian
# - changelog
# - compat
# - control
# - copyright
# - dirs
# - docs
# - rules         
#
# the following files shoud be generated at make-dist
# - files
#
# Package build process
#
# 1. edit "changelog" file with appropriate package version number
#    like "1.1.0-2." This version number will be used for actual 
#    deb package files.
#
# 2. Check permissions of the parent directory of distribution sourcecode
#    extracted directory. (ex. parent of OpenRTM-aist-1.0.0)
#    Package build script create deb packages there.
#
# 3. Run package build script debpkg_build.sh
#    This script do everithings.
#

export PATH=/usr/local/bin:/usr/bin:/bin:/usr/X11R6/bin:/usr/local/X11R6/bin:/usr/local/sbin:/usr/sbin:/sbin
export LANG=C
export LC_ALL=C

# system information
os=`uname -s`
release=`uname -r`-`uname -p`

dist_name=""
dist_key=""

# not support multiarch
not_multiarch_cnmaes="lucid marveric squeeze natty oneiric"

#---------------------------------------
# Debianコードネーム取得
#---------------------------------------
check_codename ()
{
    cnames="sarge etch lenny squeeze wheezy"
    for c in $cnames; do
	if test -f "/etc/apt/sources.list"; then
	    res=`grep $c /etc/apt/sources.list`
	else
	    echo "This distribution may not be debian/ubuntu."
	    exit
	fi
	if test ! "x$res" = "x" ; then
	    DISTRIB_CODENAME=$c
	fi
    done
}

# Check the lsb distribution name
if test -f /etc/lsb-release ; then
    . /etc/lsb-release
    if test "x$DISTRIB_DESCRIPTION" != "x" ; then
	dist_name=$DISTRIB_DESCRIPTION-`uname -m`
	dist_key=$DISTRIB_ID
    fi
fi
# Check the Fedora version
if test "x$dist_name" = "x" && test -f /etc/fedora-release ; then
    dist_name=`cat /etc/fedora-release`-`uname -m`
    dist_key=`sed -e 's/.[^0-9]*\([0-9]\).*/fc\1/' /etc/fedora-release`
fi
# Check the Debian version
if test "x$dist_name" = "x" && test -f /etc/debian_version ; then
    dist_name="Debian"`cat /etc/debian_version`-`uname -m`
    dist_key="Debian"
    check_codename
fi
# Check the Vine version
if test "x$dist_name" = "x" && test -f /etc/vine-release ; then
    dist_name=`cat /etc/vine-release`-`uname -m`
    dist_key=`sed -e 's/.*\([0-9]\)\.\([0-9]\).*/vl\1\2/' /etc/vine-release`
fi
# Check the TuboLinux version
if test "x$dist_name" = "x" && test -f /etc/turbolinux-release ; then
    dist_name=`cat /etc/tubolinux-release`-`uname -m`
    dist_key=""
fi

if test "x$dist_name" = "x" ; then
    dist_name=$os$release
fi
# Check the RedHat/Fedora version
if test "x$dist_name" = "x" && test -f /etc/redhat-release ; then
    dist_name=`cat /etc/redhat-release`-`uname -m`
fi

# only fedora and vine
if test ! "x$dist_key" = "xDebian" -a ! "x$dist_key" = "xUbuntu" ; then
    echo $dist_key
    echo "This is not debian/ubuntu"
    exit 0
fi

#------------------------------------------------------------
# create "files" file
#------------------------------------------------------------
#if test ! -f "files" ; then
#    PKGVER=`head -n 1 changelog | sed 's/.*(\([0-9\.\-]*\).*/\1/'`
#    echo "openrtm-aist_"${PKGVER}"_amd64.deb main extra" > files
#    echo "openrtm-aist-dev_"${PKGVER}"_amd64.deb main extra" >> files
#    echo "openrtm-aist-example_"${PKGVER}"_amd64.deb main extra" >> files
#    echo "openrtm-aist-doc_"${PKGVER}"_all.deb main extra" >> files
#fi

#------------------------------------------------------------
# package build process
#------------------------------------------------------------
packagedir=`pwd`/../../
rm -f $packagedir/packages/openrtm-aist*

cp -r debian $packagedir

# check multiarch support
multiarch_flg="OFF"
if test "x$dist_key" = "xDebian" || test "x$dist_key" = "xUbuntu" ; then
    for c in $not_multiarch_cnmaes; do
        if test $DISTRIB_CODENAME = $c ; then
            mv $packagedir/debian/compat /tmp/compat.$$
            echo 7 >  $packagedir/debian/compat
            mv $packagedir/debian/rules /tmp/rules.$$
            cp $packagedir/debian/rules.not-multiarch $packagedir/debian/rules
            DEB_HOST_ARCH=`dpkg-architecture -qDEB_HOST_ARCH`
            if test "x$DEB_HOST_ARCH" = "xamd64" ; then
                sed -i -s 's/lib-arch/lib64/' $packagedir/debian/rules
            else
                sed -i -s 's/lib-arch/lib/' $packagedir/debian/rules
            fi
            mv $packagedir/debian/control /tmp/control.$$
            cp $packagedir/debian/control.not-multiarch $packagedir/debian/control
            multiarch_flg="ON"
            echo "... Multiarch not supported."
            break
        fi
    done
fi

chmod 755 $packagedir/debian/rules

cd $packagedir
rm -f config.status
dpkg-buildpackage -W -us -uc -rfakeroot

mv $packagedir/../openrtm-aist* $packagedir/packages/
if test "x$multiarch_flg" = "xON" ; then 
    mv /tmp/compat.$$ $packagedir/debian/compat
    mv /tmp/rules.$$ $packagedir/debian/rules
    mv /tmp/control.$$ $packagedir/debian/control
fi
