#!/bin/sh
#
# @file pkg_install_vl4.sh
# @brief OpenRTM-aist dependent packages install script for Vine Linux 4.0
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
#


rpm_dir='/root/rpm/RPMS/i386'

libboost_regex_devel=`rpm -qa libboost-regex-devel`
libboost_devel=`rpm -qa libboost-devel`

ace=`rpm -qa ace`
ace_devel=`rpm -qa ace-devel`

### ソースRPMファイルの取得とリビルドにとても時間がかかるため、一時的に筑波大大原氏のHP[RTMインストール]からrpmパッケージをダウンロードしている。
ace_pkg="http://www.google.co.jp/url?sa=t&ct=res&cd=2&url=http%3A%2F%2Ff38.aaa.livedoor.jp%2F~kencyo%2Findex.php%3Fplugin%3Dattach%26openfile%3Dace-5.5.4-2.i386.rpm%26refer%3DRTM%25A5%25A4%25A5%25F3%25A5%25B9%25A5%25C8%25A1%25BC%25A5%25EB&ei=ehnARZOPF4OosAKS0ICGCA&usg=__QIYmPcnVjpcDLHfJHXqKGS950K0=&sig2=l1OpHCZ_vF_BTYsRPr6BDw"

### ソースRPMファイルの取得とリビルドにとても時間がかかるため、一時的に筑波大大原氏のHP[RTMインストール]からrpmパッケージをダウンロードしている。
ace_devel_pkg="http://f38.aaa.livedoor.jp/~kencyo/index.php?plugin=attach&openfile=ace-devel-5.5.4-2.i386.rpm&refer=RTM%A5%A4%A5%F3%A5%B9%A5%C8%A1%BC%A5%EB"

omniorb=`rpm -qa omniORB`
omniorbpy=`rpm -qa omniORBpy`

omniorb_pkg="http://opensource.nederland.net/omniORB/downloads/4.0.7/SRPMS/omniORB-4.0.7-2.src.rpm"
omniorbpy_pkg="http://opensource.nederland.net/omniORB/downloads/4.0.7/SRPMS/omniORBpy-2.7-1.src.rpm"

# updating apt-get
apt-get update

# installing openssl
apt-get install openssl

# installing openssl-devel
apt-get install openssl-devel

# installing python-devel
apt-get install python-devel


if test "x$libboost_devel" = "x" ; then
	echo "libboost-devel is not installed."
	echo "Installing libboost-devel"
	apt-get install libboost-devel
	echo "done"
else
	echo "libboost-devel is already installed."
fi

if test "x$libboost_regex_devel" = "x" ; then
	echo "libboost-regex-devel is not installed."
	echo "Installing libboost-regex-devel"
	apt-get install libboost-regex-devel
	echo "done"
else
	echo "libboost-regex-devel is already installed."
fi
 
if test "x$ace" = "x" ; then
	echo "ace is not installed."
	echo "downloading ace...."
	wget -O ace-5.5.4-2.i386.rpm $ace_pkg
	echo "Installing ace...."
	rpm -ivh ace-5.5.4-2.i386.rpm
	echo "done"
else
	echo "ace is already installed."
fi

if test "x$ace_devel" = "x" ; then
	echo "ace-devel is not installed."
	echo "downloading ace-devel...."
	wget -O ace-devel-5.5.4-2.i386.rpm $ace_devel_pkg
	echo "Installing ace-devel...."
	rpm -ivh ace-devel-5.5.4-2.i386.rpm
	echo "done"
else
	echo "ace-devel is already installed."
fi


if test "x$omniorb" = "x" ; then
	echo "omniORB is not installed."
	if ! [ -f $rpm_dir"/omniORB-4.0.7-2.i386.rpm" ] ; then
		echo "downloading omniORB...."
		mkdir -p /usr/src/vine/SOURCES
		(cd /usr/src/vine/SOURCES ; wget $omniorb_pkg )
		echo "Rebuilding omniORB...."
		(cd /usr/src/vine/SOURCES ; rpmbuild --rebuild omniORB-4.0.7-2.src.rpm )
	else
		echo "Pre-build package was found."
	fi
	echo "Installing omniORB...."
	( cd $rpm_dir ; rpm -ivh omniORB-4.0.7-2.i386.rpm )
	( cd $rpm_dir ; rpm -ivh omniORB-devel-4.0.7-2.i386.rpm )
	( cd $rpm_dir ; rpm -ivh omniORB-doc-4.0.7-2.i386.rpm )
	( cd $rpm_dir ; rpm -ivh omniORB-servers-4.0.7-2.i386.rpm )
	( cd $rpm_dir ; rpm -ivh omniORB-utils-4.0.7-2.i386.rpm )
	( cd $rpm_dir ; rpm -ivh omniORB-bootscripts-4.0.7-2.i386.rpm )
	echo "Making /var/omninames"
	mkdir -p /var/omninames
	echo "done"
else
	echo "omniORB is already installed."
fi

if test "x$omniorbpy" = "x" ; then
	echo "omniORBpy is not installed."

	if ! [ -f $rpm_dir"/omniORBpy-2.7-1.i386.rpm" ] ; then
	    echo "downloading omniORBpy...."
	    (cd /usr/src/vine/SOURCES ; wget $omniorbpy_pkg )
	    echo "Rebuilding omniORBpy...."
	    (cd /usr/src/vine/SOURCES ; rpmbuild --rebuild omniORBpy-2.7-1.src.rpm )
	else
	    echo "Pre-build package was found."
	fi
	echo "Installing ommniORBpy...."
	(cd $rpm_dir ; rpm -ivh omniORBpy-2.7-1.i386.rpm)
	(cd $rpm_dir ; rpm -ivh omniORBpy-devel-2.7-1.i386.rpm)
	(cd $rpm_dir ; rpm -ivh omniORBpy-doc-2.7-1.i386.rpm)
	(cd $rpm_dir ; rpm -ivh omniORBpy-standard-2.7-1.i386.rpm)
	echo "done"
else
	echo "omniORBpy is already installed."
fi
