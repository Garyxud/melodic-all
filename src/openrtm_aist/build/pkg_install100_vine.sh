#!/bin/sh
#
# @file pkg_install_vine.sh
# @brief OpenRTM-aist dependent packages installation script for Vine Linux
# @author Noriaki Ando <n-ando@aist.go.jp>
#         Shinji Kurihara
#         Tetsuo Ando
#
# このシェルスクリプトは、aceおよびomniORBのパッケージをインストールし、
# Vine Linuxの開発環境を構築します。
#
#---------------------------------------
# Vineバージョン特定
#---------------------------------------
vinever=`cat /etc/vine-release | sed 's/.*\([0-9].[0-9]\).*/\1/'`

#---------------------------------------
# リポジトリサーバ
#---------------------------------------
openrtm_repo="rpm     http://$reposerver/pub/Linux/Vine/apt $vinever/\$(ARCH) main"
default_reposerver="openrtm.org"
reposervers="openrtm.org"
reposerver=""

#---------------------------------------
# パッケージリスト
#---------------------------------------
omni="omniORB omniORB-devel omniORB-servers omniORB-doc"
ace="ace ace-devel"
openrtm="OpenRTM-aist OpenRTM-aist-devel OpenRTM-aist-doc OpenRTM-aist-example PyYAML"
openrtm04="OpenRTM-aist=0.4.2 OpenRTM-aist-devel=0.4.2 OpenRTM-aist-doc=0.4.2 OpenRTM-aist-example=0.4.2 PyYAML"
libuuid="e2fsprogs e2fsprogs-devel"
packages="$libuuid $omni $openrtm"
#packages_check="$libuuid $omni $ace $openrtm"



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
# ソースリスト更新関数の定義
#---------------------------------------
update_source_list () {
    rtmsite=`grep $reposerver /etc/apt/sources.list`
    if test "x$rtmsite" = "x" ; then
	echo "OpenRTM-aist のリポジトリが登録されていません。"
	echo "Source.list に OpenRTM-aist のリポジトリ: "
	echo "  " $openrtm_repo
	read -p "を追加します。よろしいですか？ (y/n) [y] " kick_shell

	if test "x$kick_shell" = "xn" ; then
	    echo "中断します。"
	    exit 0
	else
	    echo "rpm     http://$reposerver/pub/Linux/Vine/apt $vinever/\$(ARCH) main" >> /etc/apt/sources.list
	fi
    fi
}

#----------------------------------------
# チェック
#----------------------------------------

#----------------------------------------
# パッケージをインストールする
#----------------------------------------
install_packages () {
    for p in $*; do
	if test "x$p" = "x0.4.2" || test "x$p" = "x0.4.2" ; then
	    :
	else
	    if echo "$p" | grep -q '=0.4.2' ; then
		str=`echo "$p" |sed 's/=0.4.2//'`
	    else 
		str="$p"
	    fi

	    ins=`rpm -qa $str`

	    if test "x$ins" = "x"; then
		echo "Now installing: " $p
		apt-get install $p
		echo "done."
		echo ""
	    else  
		if echo "$ins" |grep -q '0.4.2-0' ; then

		    if echo "$p" | grep -q '=0.4.2'; then
			echo $ins
			echo $str"-0.4.2 is already installed"
			echo ""
		    else 
			apt-get install $p
			echo "done." 
			echo ""
		    fi

		elif echo "$ins" |grep -q '1.0.0-0'; then 
		        echo $ins
			echo $str"-1.0.0 is already installed"
			echo ""

		    if echo "$p" | grep -q '=0.4.2'; then
		       apt-get install $p
		       echo "done." 
		       echo ""
		    fi
	       else 
 		    echo $ins
		    echo $str "is already installed."
		    echo ""
		fi
	    fi
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
	echo "rpm -e $p"
	echo "done."
	echo ""
    done
}



#---------------------------------------
# メイン
#---------------------------------------
check_root


if test "x$1" = "x0.4.2" || test "x$1" = "x0.4" ; then
    openrtm=$openrtm04
    packages="$libuuid $omni $ace $openrtm"
    #packages="$openrtm"
fi

if test "x$1" = "x-u" ; then
    uninstall_packages `reverse $packages`
else
    check_reposerver
    update_source_list
    apt-get update
    install_packages $1 $packages
fi