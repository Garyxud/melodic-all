#!/bin/sh
#
# @file pkg_install_ubuntu.sh
# @brief OpenRTM-aist dependent packages install script for Debian-sarge
# @author Noriaki Ando <n-ando@aist.go.jp>
#         Shinji Kurihara
#         Tetsuo Ando
#         Harumi Miyamoto
#         Seisho Irie
#

#
# sudo sh pkg_install_ubuntu.sh [-s/-us/-d/-ud/-c/-uc/-r/-ur]
# option -s            : install tool_packages for build Source packages 
# option -us           : Uninstall tool_packages for build Source packages 
# option -d            : install tool_packages for build Distribution
# option -ud           : Uninstall tool_packages for build Distribution
# option -c            : install robot Component developer tool_packages
# option -uc           : Uninstall robot Component developer tool_packages
# option -r            : install robot component Runtime tool_packages
# option -ur           : Uninstall robot component Runtime tool_packages
#

#---------------------------------------
# パッケージリスト
#---------------------------------------
omni="libomniorb4-dev omniidl"
ace="libace libace-dev"
openrtm="openrtm-aist openrtm-aist-doc openrtm-aist-dev openrtm-aist-example python-yaml"
openrtm04="openrtm-aist=0.4.2-1 openrtm-aist-doc=0.4.2-1 openrtm-aist-dev=0.4.2-1 openrtm-aist-example=0.4.2-1 python-yaml"
devel="gcc g++ make uuid-dev libboost-filesystem-dev"
packages="$devel $omni $openrtm"
u_packages="$omni $ace $openrtm "

default_reposerver="openrtm.org"
reposervers="openrtm.org"
reposerver=""

#---------------------------------------
autotools="autoconf libtool libtool-bin"
build_tools="gcc g++ make"
dep_pkg="uuid-dev doxygen"
deb_tools="build-essential debhelper devscripts"
omni_devel="libomniorb4-dev omniidl omniorb-nameserver"
omni_runtime="omniorb-nameserver"
openrtm_devel="openrtm-aist-doc openrtm-aist-dev python-yaml"
openrtm_runtime="openrtm-aist openrtm-aist-example"

source_tools="subversion texlive texlive-lang-cjk xdvik-ja python-yaml"

source_pkgs="$autotools $build_tools $dep_pkg $source_tools $omni_devel"
u_source_pkgs="$autotools $build_tools $dep_pkg $source_tools $omni_devel"

distribution_pkgs="$build_tools $dep_pkg $deb_tools $omni_devel"
u_distribution_pkgs="$build_tools $dep_pkg $deb_tools $omni_devel"

component_pkgs="$build_tools $dep_pkg $omni_devel $openrtm_devel $openrtm_runtime"
u_component_pkgs="$build_tools $dep_pkg $omni_devel $openrtm_devel $openrtm_runtime"

runtime_pkgs="$omni_runtime $openrtm_runtime"
u_runtime_pkgs="$omni_runtime $openrtm_runtime"

#---------------------------------------
# ロケールの言語確認
#---------------------------------------
check_lang()
{
lang="en"

locale | grep ja_JP > /dev/null && lang="jp"

if test "$lang" = "jp" ;then
    msg1="ディストリビューションを確認してください。\nDebianかUbuntu以外のOSの可能性があります。"
    msg2="コードネーム :"
    msg3="このOSはサポートしておりません。"
    msg4="OpenRTM-aist のリポジトリが登録されていません。"
    msg5="Source.list に OpenRTM-aist のリポジトリ: "
    msg6="を追加します。よろしいですか？(y/n)[y] "
    msg7="中断します。"
    msg8="ルートユーザーで実行してください。"
    msg9="インストール中です..."
    msg10="完了"
    msg11="アンインストール中です."
else
    msg1="This distribution may not be debian/ubuntu."
    msg2="The code name is : "
    msg3="This OS is not supported."
    msg4="No repository entry for OpenRTM-aist is configured in your system."
    msg5="repository entry for OpenRTM-aist: "
    msg6="Do you want to add new repository entry for OpenRTM-aist in source.list? (y/n) [y] "
    msg7="Abort."
    msg8="This script should be run as root."
    msg9="Now installing: "
    msg10="done."
    msg11="Now uninstalling: "
fi

}

#----------------------------------------
# 近いリポジトリサーバを探す
#----------------------------------------
check_reposerver()
{
    minrtt=65535
    nearhost=''
    for host in $reposervers; do
	rtt=`ping -c 1 $host | grep 'time=' | sed -e 's/^.*time=\([0-9\.]*\) ms.*/\1/' 2> /dev/null`
	if test "x$rtt" = "x"; then
	    rtt=65535
	fi
	if test `echo "scale=2 ; $rtt < $minrtt" | bc` -gt 0; then
	    minrtt=$rtt
	    nearhost=$host
	fi
    done
    if test "x$nearhost" = "x"; then
	echo "Repository servers unreachable.", $hosts
	echo "Check your internet connection. (or are you using proxy?)"
	nearhost=$default_reposerver
    fi
    reposerver=$nearhost
}


#---------------------------------------
# リポジトリサーバ
#---------------------------------------
create_srclist () {
    codename=`sed -n /DISTRIB_CODENAME=/p /etc/lsb-release`
    cnames=`echo "$codename" | sed 's/DISTRIB_CODENAME=//'`
    #cnames="sarge edgy feisty gutsy hardy intrepid"
    for c in $cnames; do
	if test -f "/etc/apt/sources.list"; then
	    res=`grep $c /etc/apt/sources.list`
	else
	    echo $msg1
	    exit
	fi
	if test ! "x$res" = "x" ; then
	    code_name=$c
	fi
    done
    if test ! "x$code_name" = "x"; then
	echo $msg2 $code_name
    else
	echo $msg3
	exit
    fi
    openrtm_repo="deb http://$reposerver/pub/Linux/ubuntu/ $code_name main"
}

#---------------------------------------
# ソースリスト更新関数の定義
#---------------------------------------
update_source_list () {
    rtmsite=`grep "$openrtm_repo" /etc/apt/sources.list`
    if test "x$rtmsite" = "x" ; then
	echo $msg4
	echo $msg5
	echo "  " $openrtm_repo
	read -p "$msg6" kick_shell

	if test "x$kick_shell" = "xn" ; then
	    echo $msg7
	    exit 0
	else
	    echo $openrtm_repo >> /etc/apt/sources.list
	fi
    fi
}

#----------------------------------------
# root かどうかをチェック
#----------------------------------------
check_root () {
    if test ! `id -u` = 0 ; then
	echo ""
	echo $msg8
	echo $msg7
	echo ""
	exit 1
    fi
}

#----------------------------------------
# パッケージインストール関数
#----------------------------------------
install_packages () {
    for p in $*; do
	echo $msg9 $p
	apt-get install --force-yes -y $p
	echo $msg10
	echo ""
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
	echo $msg11 $p
	aptitude remove $p
	if test "$?" != 0; then
            apt-get purge $p
	fi
	echo $msg10
	echo ""
    done
}

#---------------------------------------
# install tool_packages for build Source packages 
#---------------------------------------
install_source_packages(){
    echo "--------install $source_pkgs"
    install_packages $source_pkgs
}
#---------------------------------------
# Uninstall tool_packages for build Source packages 
#---------------------------------------
uninstall_source_packages(){
    echo "--------uninstall $u_source_pkgs"
    uninstall_packages `reverse $u_source_pkgs`
}
#---------------------------------------
# install tool_packages for build Distribution
#---------------------------------------
install_distribution(){
    echo "--------install $distribution_pkgs"
    install_packages $distribution_pkgs
}
#---------------------------------------
# Uninstall tool_packages for build Distribution
#---------------------------------------
uninstall_distribution(){
    echo "--------uninstall $u_distribution_pkgs"
    uninstall_packages `reverse $u_distribution_pkgs`
}
#---------------------------------------
# install robot Component developer tool_packages
#---------------------------------------
install_componet_developer(){
    echo "--------install $component_pkgs()"
    install_packages $component_pkgs
}
#---------------------------------------
# Uninstall robot Component developer tool_packages
#---------------------------------------
uninstall_componet_developer(){
    echo "--------uninstall $u_component_pkgs"
    uninstall_packages `reverse $u_component_pkgs`
}
#---------------------------------------
# install robot component Runtime tool_packages
#---------------------------------------
install_runtime(){
    echo "--------install $runtime_pkgs"
    install_packages $runtime_pkgs
}
#---------------------------------------
# Uninstall robot component Runtime tool_packages
#---------------------------------------
uninstall_runtime(){
    echo "--------uninstall $u_runtime_pkgs"
    uninstall_packages `reverse $u_runtime_pkgs`
}
#---------------------------------------
#
#---------------------------------------
howto_usage(){
    echo "Usage: sudo sh pkg_install_ubuntu.sh [-s/-us/-d/-ud/-c/-uc/-r/-ur]"
    echo "       option -s            : install tool_packages for build Source packages"
    echo "       option -us           : Uninstall tool_packages for build Source packages"
    echo "       option -d            : install tool_packages for build Distribution"
    echo "       option -ud           : Uninstall tool_packages for build Distribution"
    echo "       option -c            : install robot Component developer tool_packages"
    echo "       option -uc           : Uninstall robot Component developer tool_packages"
    echo "       option -r            : install robot component Runtime tool_packages"
    echo "       option -ur           : Uninstall robot component Runtime tool_packages"
}

#---------------------------------------
# メイン
#---------------------------------------
check_lang
check_root
check_reposerver
create_srclist
update_source_list
apt-get autoclean
apt-get update

if test "x$1" = "x-s" ; then
    install_source_packages
    exit 0
fi
if test "x$1" = "x-us" ; then
    uninstall_source_packages
    exit 0
fi
if test "x$1" = "x-d" ; then
    install_distribution
    exit 0
fi
if test "x$1" = "x-ud" ; then
    uninstall_distribution
    exit 0
fi
if test "x$1" = "x-c" ; then
    install_componet_developer
    exit 0
fi
if test "x$1" = "x-uc" ; then
    uninstall_componet_developer
    exit 0
fi
if test "x$1" = "x-r" ; then
    install_runtime
    exit 0
fi
if test "x$1" = "x-ur" ; then
    uninstall_runtime
    exit 0
fi

howto_usage
