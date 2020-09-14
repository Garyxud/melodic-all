#!/bin/sh
#
# @file pkg_install_fedora.sh
# @brief OpenRTM-aist dependent packages install script for Fedora
# @author Noriaki Ando <n-ando@aist.go.jp>
#         Shinji Kurihara
#         Tetsuo Ando
#
# このシェルスクリプトは、aceおよびomniORBのパッケージをインストールし、
# fedoraの開発環境を構築します。
#
# $Id$
#

#---------------------------------------
# パッケージリスト
#---------------------------------------
omni="omniORB omniORB-devel omniORB-doc omniORB-servers omniORB-utils omniORB-bootscripts"
ace="ace ace-devel"
openrtm="OpenRTM-aist OpenRTM-aist-devel OpenRTM-aist-doc OpenRTM-aist-example PyYAML"
openrtm04="OpenRTM-aist-0.4.2 OpenRTM-aist-devel-0.4.2 OpenRTM-aist-doc-0.4.2 OpenRTM-aist-example-0.4.2 PyYAML"
packages="gcc-c++ libuuid-devel $omni $ace $openrtm"


#----------------------------------------
# root かどうかをチェック
#----------------------------------------
check_root () {
    if test ! `id -u` = 0 ; then
	echo ""
	echo "This script should be run by root user."
	echo "Abort."
	echo ""
	exit 1
    fi
}

#---------------------------------------
# インストール済パッケージリスト
#---------------------------------------
rpm_qa="/tmp/yum_list.txt"
get_pkg_list () {
    rpm -qa > $rpm_qa
}
clean_pkg_list () {
    rm -f $rpm_qa
}

#---------------------------------------
# リポジトリサイト設定ファイルを生成
#---------------------------------------
openrtm_repo () {
cat <<EOF
[openrtm]
name=Fedora \$releasever - \$basearch
failovermethod=priority
baseurl=http://www.openrtm.org/pub/Linux/Fedora/releases/\$releasever/Fedora/\$basearch/os/Packages
enabled=1
gpgcheck=0
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-fedora file:///etc/pki/rpm-gpg/RPM-GPG-KEY
EOF
} 
create_repo() {
    repo="/etc/yum.repos.d/openrtm.repo"
    if test ! -f $repo ; then
	echo "OpenRTM-aist のリポジトリが登録されていません。"
	echo "OpenRTM-aist のリポジトリ: "
	echo "  http://www.openrtm.org/pub/Linux/Fedora/"
	read -p "を追加します。よろしいですか？ (y/n) [y] " kick_shell

	if test "x$kick_shell" = "xn" ; then
	    echo "中断します。"
	    exit 0
	else
	    openrtm_repo > /etc/yum.repos.d/openrtm.repo
	fi
    fi
}

#----------------------------------------
# パッケージインストール関数
#----------------------------------------
install_packages () {
    for p in $*; do
	ins=`rpm -qa $p`
	if test "x$ins" = "x"; then
	    echo "Now installing: " $p
	    yum install $p
	    echo "done."
	    echo ""
	else
	    echo $p "is already installed."
	    echo ""
	fi
    done
}

#------------------------------------------------------------
# リストを逆順にする
#------------------------------------------------------------
reverse () {
    for i in $*; do
	echo $i
    done | sed '1!G;h;$!d'
}

#----------------------------------------
# パッケージをアンインストールする
#----------------------------------------
uninstall_packages () {
    for p in $*; do
	echo "Now uninstalling: " $p
	yum erase $p
	echo "done."
	echo ""
    done
}

#---------------------------------------
# メイン
#---------------------------------------
check_root

#if test "x$1" = "x0.4.2" || test "x$1" = "x0.4" ; then
    openrtm=$openrtm04
    packages="$devel $omni $ace $pyyaml $openrtm"
#fi

if test "x$1" = "x-u" ; then
    uninstall_packages `reverse $packages`
else
    create_repo
    install_packages $packages
fi
