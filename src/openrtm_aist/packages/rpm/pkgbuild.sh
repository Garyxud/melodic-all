#!/bin/sh 

openrtm_spec_old () {
cat <<EOF
#------------------------------------------------------------
#
# @file RPM spec file for OpenRTM-aist
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
%define pkgname OpenRTM-aist
%define version        1.0.0
%define distname       __DISTNAME__
%define builddir       %{_topdir}/BUILD/%{distname}
%define pkgver         1
%define _unpackaged_files_terminate_build   1
%define __os_install_post %{nil}

#------------------------------------------------------------
# Package information
Name:    OpenRTM-aist
Version: %{version}
Release: %{pkgver}.%{distname}
Summary: OpenRTM-aist: RT-Component development environment
Group:   Development/Libraries
License: LGPL
URL:     http://www.is.aist.go.jp/rt/OpenRTM-aist/
Source0: %{pkgname}-%{version}-RELEASE.tar.gz
Vendor:  AIST

#------------------------------------------------------------
# Build environment
Prefix:        /usr
Buildroot:     %{_tmppath}/%{pkgname}-%{version}-%{release}-root
Requires:      omniORB
Requires:      omniORB-servers
#Requires:      ace >= 5.4.0
BuildRequires: omniORB-devel
#BuildRequires: ace-devel >= 5.4.0
BuildRequires: python

%description
OpenRTM-aist is a reference implementation of RTC (Robotic Technology 
Component) specification which is OMG standard. OpenRTM-aist includes
RT-Middleware runtime environment and RTC framework. The OMG standard
defines a component model and certain important infrastructure services
applicable to the domain of robotics software development.
OpenRTM-aist is being developed and distributed by 
Task Intelligence Research Group, Intelligent Systems Research Institute, 
National Institute of Advanced Industrial Science and Technology (AIST), Japan. 
Please see http://www.is.aist.go.jp/rt/OpenRTM-aist/html/ for more detail. 

#------------------------------------------------------------
# devel package
%package devel
Summary: Header files
Group: Development/Libraries
%description devel
The header files and libraries needed for developing programs using
OpenRTM-aist.

#------------------------------------------------------------
# doc package
%package doc
Summary: Documentation
Group: Development/Libraries
%description doc
Developer documentation.

#------------------------------------------------------------
# example package
%package example
Summary: Example
Group: Development/Libraries
%description example
Example components and sources

#------------------------------------------------------------
# prep section
%prep
%{__rm} -rf %{buildroot}
%setup -n %{pkgname}-%{version} -q

#------------------------------------------------------------
# build section
%build
%configure --prefix=/usr
%{__make}

#------------------------------------------------------------
# install section
%install
export DONT_STRIP=1
%{__make} DESTDIR=%{buildroot} install

#------------------------------------------------------------
# clean section
%clean
%{__rm} -rf %{buildroot}

#------------------------------------------------------------
# files section
%files
/etc/rtc.conf.sample
%defattr(-,root,root,-)
%{_libdir}/libRTC*
%{_libdir}/libcoil*

#------------------------------------------------------------
# devel package file list
%files devel
%defattr(-,root,root,-)
%attr(755,root,root) %{_bindir}/*
%{_includedir}/*
%{_libdir}/OpenRTM-aist/*
%{_libdir}/pkgconfig/*

#------------------------------------------------------------
# doc package file list
%files doc
%defattr(-,root,root,-)
%{_datadir}/OpenRTM-aist/docs/*

#------------------------------------------------------------
# example package file list
%files example
%defattr(-,root,root,-)
%attr(755,root,root) %{_datadir}/OpenRTM-aist/examples/*Comp*
%{_datadir}/OpenRTM-aist/examples/*.conf
%{_datadir}/OpenRTM-aist/examples/*.conf.org
%{_datadir}/OpenRTM-aist/examples/src/*
%{_datadir}/OpenRTM-aist/examples/rtcs/*
%{_datadir}/OpenRTM-aist/examples/templates/*

#------------------------------------------------------------
# changelog section
%changelog
* Thu Sep 27 2007 Noriaki Ando <n-ando@aist.go.jp> - 0.4.1-1._distname
- The second public release version of OpenRTM-aist-0.4.1.
EOF
} 
openrtm_spec () {
cat <<EOF
#------------------------------------------------------------
#
# @file RPM spec file for OpenRTM-aist
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
%define pkgname OpenRTM-aist
%define version        1.0.0
%define distname       __DISTNAME__
%define builddir       %{_topdir}/BUILD/%{distname}
%define pkgver         1
%define _unpackaged_files_terminate_build   1

#------------------------------------------------------------
# Package information
Name:    OpenRTM-aist
Version: %{version}
Release: %{pkgver}.%{distname}
Summary: OpenRTM-aist: RT-Component development environment
Group:   Development/Libraries
License: LGPL
URL:     http://www.is.aist.go.jp/rt/OpenRTM-aist/
Source0: %{pkgname}-%{version}-RELEASE.tar.gz
Vendor:  AIST

#------------------------------------------------------------
# Build environment
Prefix:        /usr
Buildroot:     %{_tmppath}/%{pkgname}-%{version}-%{release}-root
Requires:      libomniORB4.1
Requires:      omniORB-servers
#Requires:      ace >= 5.4.0
BuildRequires: libomniORB4.1-devel
#BuildRequires: ace-devel >= 5.4.0
BuildRequires: python

%description
OpenRTM-aist is a reference implementation of RTC (Robotic Technology 
Component) specification which is OMG standard. OpenRTM-aist includes
RT-Middleware runtime environment and RTC framework. The OMG standard
defines a component model and certain important infrastructure services
applicable to the domain of robotics software development.
OpenRTM-aist is being developed and distributed by 
Task Intelligence Research Group, Intelligent Systems Research Institute, 
National Institute of Advanced Industrial Science and Technology (AIST), Japan. 
Please see http://www.is.aist.go.jp/rt/OpenRTM-aist/html/ for more detail. 

#------------------------------------------------------------
# devel package
%package devel
Summary: Header files
Group: Development/Libraries
%description devel
The header files and libraries needed for developing programs using
OpenRTM-aist.

#------------------------------------------------------------
# doc package
%package doc
Summary: Documentation
Group: Development/Libraries
%description doc
Developer documentation.

#------------------------------------------------------------
# example package
%package example
Summary: Example
Group: Development/Libraries
%description example
Example components and sources

#------------------------------------------------------------
# prep section
%prep
%{__rm} -rf %{buildroot}
%setup -n %{pkgname}-%{version} -q

#------------------------------------------------------------
# build section
%build
%configure --prefix=/usr
%{__make}

#------------------------------------------------------------
# install section
%install
%{__make} DESTDIR=%{buildroot} install

#------------------------------------------------------------
# clean section
%clean
%{__rm} -rf %{buildroot}

#------------------------------------------------------------
# files section
%files
/etc/rtc.conf.sample
%defattr(-,root,root,-)
%{_libdir}/libRTC*
%{_libdir}/libcoil*

#------------------------------------------------------------
# devel package file list
%files devel
%defattr(-,root,root,-)
%attr(755,root,root) %{_bindir}/*
%{_includedir}/*
%{_libdir}/OpenRTM-aist/*
%{_libdir}/pkgconfig/*

#------------------------------------------------------------
# doc package file list
%files doc
%defattr(-,root,root,-)
%{_datadir}/OpenRTM-aist/docs/*

#------------------------------------------------------------
# example package file list
%files example
%defattr(-,root,root,-)
%attr(755,root,root) %{_datadir}/OpenRTM-aist/examples/*Comp*
%{_datadir}/OpenRTM-aist/examples/*.conf
%{_datadir}/OpenRTM-aist/examples/*.conf.org
%{_datadir}/OpenRTM-aist/examples/src/*
%{_datadir}/OpenRTM-aist/examples/rtcs/*
%{_datadir}/OpenRTM-aist/examples/templates/*

#------------------------------------------------------------
# changelog section
%changelog
* Thu Sep 27 2007 Noriaki Ando <n-ando@aist.go.jp> - 0.4.1-1._distname
- The second public release version of OpenRTM-aist-0.4.1.
EOF
} 

pyyaml_spec () {
cat <<EOF
%define python_sitelib %(%{__python} -c 'from distutils import sysconfig; print sysconfig.get_python_lib()')

%define distname  __DISTNAME__
%define real_name PyYAML
%define pkgver    1

Summary: Python package implementing YAML parser and emitter
Name: PyYAML
Version: 3.05
Release: %{pkgver}.%{distname}
License: GPL
Group: Development/Libraries
URL: http://pyyaml.org/wiki/PyYAML

Source: http://pyyaml.org/download/pyyaml/PyYAML-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root

BuildArch: noarch
BuildRequires: python-devel >= 2.2
Requires: python >= 2.2

Provides: PyYAML = %{name}-%{version}
Obsoletes: PyYAML <= %{name}-%{version}

%description
PyYAML is a YAML parser and emitter for the Python programming language. 

YAML is a data serialization format designed for human readability
and interaction with scripting languages.

%prep
%setup -n %{real_name}-%{version}

%build
%{__python} setup.py build

%install
%{__rm} -rf %{buildroot}
%{__python} setup.py install -O1 --skip-build --root="%{buildroot}" --prefix="%{_prefix}"

%clean
%{__rm} -rf %{buildroot}

%files
%defattr(-, root, root, 0755)
%doc LICENSE README examples/
%{python_sitelib}/yaml/

%changelog
* Tue May 27 2008 Noriaki Ando <n-ando@aist.go.jp> 3.05-distname.0
- Distribution named package.
* Sun May 13 2007 Dag Wieers <dag@wieers.com> - 3.05-1
- Initial package. (using DAR)
EOF
}

export PATH=/usr/local/bin:/usr/bin:/bin:/usr/X11R6/bin:/usr/local/X11R6/bin:/usr/local/sbin:/usr/sbin:/sbin
export LANG=C
export LC_ALL=C

# date
date=`date "+%y%m%d%H%M"`
time=/usr/bin/time
# package location and build directory
package=/usr/users/builder/PackageBuild/src/OpenRTM-aist-1.0.0-RELEASE.tar.gz
packagedir=OpenRTM-aist-1.0.0
#packagedir=PyYAML-3.05
package_date=`ls -al $package | awk '{printf("%s/%s %s\n",$6,$7,$8);}'`
package_date=`diff -ac $package /dev/null | head -1 |awk '{print $3,$4,$5,$6,$7,$8;}'`
package_name=`basename $package`
buildroot=/usr/users/builder/PackageBuild

logheader="<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 3.2 Final//EN\"><html><body><pre>"
logfooter="</pre></body></html>"

# system information
hostname=`hostname`
os=`uname -s`
release=`uname -r`-`uname -p`


dist_name=""
dist_key=""
# Check the lsb distribution name
if test -f /etc/lsb-release ; then
    . /etc/lsb-release
    if test "x$DISTRIB_DESCRIPTION" != "x" ; then
	dist_name=$DISTRIB_DESCRIPTION-`uname -m`
    fi
fi
# Check the Fedora version
if test "x$dist_name" = "x" && test -f /etc/fedora-release ; then
    dist_name=`cat /etc/fedora-release`-`uname -m`
    dist_key=`sed -e 's/.[^0-9]*\([0-9]\+\).*/fc\1/' /etc/fedora-release`
fi
#Check the Debian version
if test "x$dist_name" = "x" && test -f /etc/debian_version ; then
    dist_name="Debian"`cat /etc/debian_version`-`uname -m`
    dist_key=""
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
if test "x$dist_key" = "x" ; then
    exit 0
fi

distname=`echo $dist_name | sed 's/[ |\(|\)]//g'`
# system dependent build directory and log file name
builddir=$buildroot/$distname
timestamp=$buildroot/.$distname
logfile=$distname-$date.log


echo $dist_key

build=""
echo $dist_key
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
	echo "New source file was found."
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
# package build process
#------------------------------------------------------------
echo $logheader > $buildroot/make-$logfile

mkdir {BUILD,RPMS,SOURCES,SPECS,SRPMS}
mkdir RPMS/{i386,i586,i686,x86_64,noarch}
ln -s $package SOURCES/
cd SPECS
echo "-----------------------------"
echo $dist_key


if test "x$dist_key" = "xfc10" || test "x$dist_key" = "xfc11" \
|| test "x$dist_key" = "xvl40" || test "x$dist_key" = "xvl42" ; then
    openrtm_spec_old > OpenRTM-aist.spec.tmp
    echo $dist_key
else
    openrtm_spec > OpenRTM-aist.spec.tmp
    echo $dist_key
fi

#openrtm_spec > python-yaml.spec.tmp
sed "s/__DISTNAME__/$dist_key/g" OpenRTM-aist.spec.tmp > OpenRTM-aist.spec
#sed "s/__DISTNAME__/$dist_key/g" python-yaml.spec.tmp > python-yaml.spec
echo "%_topdir $builddir" > .rpmrc

rpm_def="_topdir $builddir"
if $time -p -o make_time-$logfile rpmbuild --define "$rpm_def" -ba OpenRTM-aist.spec >> $buildroot/make-$logfile 2>&1 ; then
#if $time -p -o make_time-$logfile rpmbuild --define "$rpm_def" -ba python-yaml.spec >> $buildroot/make-$logfile 2>&1 ; then
    echo $logfooter >> $buildroot/make-$logfile
    make_time=`awk '/real/{printf("%s[s] ", $0);}' make_time-$logfile`
    echo "make: OK" >> $buildroot/$logfile
    echo "make_time: $make_time" >> $buildroot/$logfile
    rm -f make_time-$logfile
else
    echo "make: NG" >> $buildroot/$logfile
    rm -f make_time-$logfile
    exit 1
fi

