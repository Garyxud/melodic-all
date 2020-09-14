#!/bin/sh

export PATH=/usr/local/bin:/usr/bin:/bin:/usr/X11R6/bin:/usr/local/X11R6/bin:/usr/local/sbin:/usr/sbin:/sbin

# date
date=`date "+%y%m%d%H%M"`
time=/usr/bin/time
# package location and build directory
package=/usr/users/n-ando/OpenRTM-aist/OpenRTM-aist-0.4.1.tar.gz
packagedir=OpenRTM-aist-0.4.1
package_date=`ls -al $package | awk '{printf("%s/%s %s\n",$6,$7,$8);}'`
package_name=`basename $package`
buildroot=/usr/users/n-ando/BuildTests

# system information
hostname=`hostname`
os=`uname -s`
release=`uname -r`

dist_name=""
# Check the RedHat/Fedora version
if test -f /etc/redhat-release ; then
    dist_name=`cat /etc/redhat-release`
fi
# Check the Fedora version
if test -f /etc/fedora-release ; then
    dist_name=`cat /etc/fedora-release`
fi
#Check the Debian version
if test -f /etc/debian_version ; then
    dist_name=`cat /etc/debian_version`
fi
# Check the Vine version
if test -f /etc/vine-release ; then
    dist_name=`cat /etc/vine-release`
fi
# Check the TuboLinux version
if test -f /etc/turbolinux-release ; then
    dist_name=`cat /etc/tubolinux-release`
fi



# system dependent build directory and log file name
builddir=$buildroot/$os$release
timestamp=$buildroot/.$os$release
logfile=$os$release-$date.log

build=""

# check package
if test -f $package ; then
    echo "Package found: " $package
else
    echo "Package not found: " $pacakge
    exit 1
fi
cd $buildroot

# check if package is new
if test -f $timestamp ; then
    if test $package -nt $timestamp ; then
	build=yes
	echo "New distribution package was found."
        touch $timestamp
    fi
else
    echo "Timestamp not found."
    touch $timestamp
    build=yes
fi

if test "x$build" = "x" ; then
    echo "No new package."
    exit 1
fi

# cleanup 
echo "cleanup " $builddir/$packagedir
rm -rf $builddir

mkdir -p $builddir
cd $builddir

echo "distribution: " $dist_name >> $buildroot/$logfile
echo "package: $package_date " >> $buildroot/$logfile

#------------------------------------------------------------
# extract process
#------------------------------------------------------------
echo 
if $time -p -o tar_time-$logfile tar xvzf $package > $buildroot/extract-$logfile 2>&1 ; then
    tar_time=`awk '/real/{printf("%s[s] ", $0);}' tar_time-$logfile`
    echo "extract: OK" >> $buildroot/$logfile
    echo "extract_time: $tar_time" >> $buildroot/$logfile
    rm -f tar_time-$logfile
else
    echo "extract: fail" >> $buildroot/$logfile
    rm -f tar_time-$logfile
    exit 1
fi

cd $packagedir


#------------------------------------------------------------
# configure process
#------------------------------------------------------------
if $time -p -o conf_time-$logfile ./configure > $buildroot/configure-$logfile 2>&1 ; then
    conf_time=`awk '/real/{printf("%s[s] ", $0);}' conf_time-$logfile`
    echo "configure: OK" >> $buildroot/$logfile
    echo "configure_time: $conf_time" >> $buildroot/$logfile
    rm -f conf_time-$logfile
else
    echo "configure: fail" >> $buildroot/$logfile
    rm -f conf_time-$logfile
    exit 1
fi

#------------------------------------------------------------
# make process
#------------------------------------------------------------
if $time -p -o make_time-$logfile make > $buildroot/make-$logfile 2>&1 ; then
    make_time=`awk '/real/{printf("%s[s] ", $0);}' make_time-$logfile`
    echo "make: OK" >> $buildroot/$logfile
    echo "make_time: $make_time" >> $buildroot/$logfile
    rm -f make_time-$logfile
else
    echo "make: NG" >> $buildroot/$logfile
    rm -f make_time-$logfile
    exit 1
fi

