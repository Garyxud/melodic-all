#!/bin/sh
#
# @file pkg_install_vl32.sh
# @brief OpenRTM-aist dependent packages install script for Vine Linux 3.2
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
#


rpm_dir='/root/rpm/RPMS/i386'


ace=`rpm -qa ace`
ace_devel=`rpm -qa ace-devel`

### ソースRPMファイルの取得とリビルドにとても時間がかかるため、[OpenRTMインストール]のページからrpmパッケージをダウンロードしている。
ace_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/ace-5.4.1-1.i386.rpm"

### ソースRPMファイルの取得とリビルドにとても時間がかかるため、[OpenRTMインストール]のページからrpmパッケージをダウンロードしている。
ace_devel_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/ace-devel-5.4.1-1.i386.rpm"

boost=`rpm -qa boost`
boost_devel=`rpm -qa boost-devel`
boost_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/boost-1.32.0-1.i386.rpm"
boost_dev_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/boost-devel-1.32.0-1.i386.rpm"

omniorb=`rpm -qa omniORB`
omniorbpy=`rpm -qa omniORBpy`
omniorb_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/omniORB-4.0.5-1.i386.rpm"
omniorb_dev_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/omniORB-devel-4.0.5-1.i386.rpm"
omniorb_doc_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/omniORB-doc-4.0.5-1.i386.rpm"
omniorbpy_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/omniORBpy-2.3-1.i386.rpm"

wxpython_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/wxPythonGTK-py2.3-2.5.1.5-1.i386.rpm"
pyxml_pkg="http://www.is.aist.go.jp/rt/OpenRTM-aist-Tutorial/packages/Vine3.0/RPMS/i386/PyXML-0.8.4-3.i386.rpm"


if test "x$boost" = "x" ; then
	echo "boost is not installed."
	echo "Installing boost"
	wget $boost_pkg
	rpm -ivh boost-1.32.0-1.i386.rpm
	echo "done"
else
	echo "boost is already installed."
fi

if test "x$boost_devel" = "x" ; then
	echo "boost-devel is not installed."
	echo "Installing boost-devel"
	wget $boost_dev_pkg
	rpm -ivh boost-devel-1.32.0-1.i386.rpm
	echo "done"
else
	echo "boost-devel is already installed."
fi


if test "x$ace" = "x" ; then
	echo "ace is not installed."
	echo "downloading ace...."
	wget $ace_pkg
	echo "Installing ace...."
	rpm -ivh ace-5.4.1-1.i386.rpm
	echo "done"
else
	echo "ace is already installed."
fi

if test "x$ace_devel" = "x" ; then
	echo "ace-devel is not installed."
	echo "downloading ace-devel...."
	wget $ace_devel_pkg
	echo "Installing ace-devel...."
	rpm -i --nodeps ace-devel-5.4.1-1.i386.rpm
	echo "done"
else
	echo "ace-devel is already installed."
fi


if test "x$omniorb" = "x" ; then
	echo "omniORB is not installed."
	echo "downloading omniORB...."
	wget $omniorb_pkg
	wget $omniorb_dev_pkg
	wget $omniorb_doc_pkg
	echo "Installing omniORB...."
	rpm -ivh omniORB-4.0.5-1.i386.rpm
	rpm -ivh omniORB-devel-4.0.5-1.i386.rpm
	rpm -ivh omniORB-doc-4.0.5-1.i386.rpm

	echo "done"
else
	echo "omniORB is already installed."
fi

if test "x$omniorbpy" = "x" ; then
	echo "omniORBpy is not installed."
	echo "downloading omniORBpy...."
	wget $omniorbpy_pkg
	echo "Installing ommniORBpy...."
	rpm -ivh omniORBpy-2.3-1.i386.rpm
	echo "done"
else
	echo "omniORBpy is already installed."
fi

echo "downloading wxPythonGTK...."
wget $wxpython_pkg
echo "Installing wxPythonGTK...."
rpm -ivh wxPythonGTK-py2.3-2.5.1.5-1.i386.rpm

echo "downloading PyXML...."
wget $pyxml_pkg
echo "Installing PyXML...."
rpm -ivh PyXML-0.8.4-3.i386.rpm