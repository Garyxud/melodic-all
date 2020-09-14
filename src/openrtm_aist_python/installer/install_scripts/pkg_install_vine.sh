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
openrtm_repo="rpm     http://www.openrtm.org/pub/Linux/Vine/apt $vinever/\$(ARCH) main"

#---------------------------------------
# パッケージリスト
#---------------------------------------
omni="omniORB omniORB-devel omniORB-servers omniORB-doc"
openrtm="OpenRTM-aist OpenRTM-aist-devel OpenRTM-aist-doc OpenRTM-aist-example PyYAML"
libuuid="e2fsprogs e2fsprogs-devel"
packages="$libuuid $omni $openrtm"




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
# ソースリスト更新関数の定義
#---------------------------------------
update_source_list () {
    rtmsite=`grep openrtm /etc/apt/sources.list`
    if test "x$rtmsite" = "x" ; then
	echo "OpenRTM-aist のリポジトリが登録されていません。"
	echo "Source.list に OpenRTM-aist のリポジトリ: "
	echo "  " $openrtm_repo
	read -p "を追加します。よろしいですか？ (y/n) [y] " kick_shell

	if test "x$kick_shell" = "xn" ; then
	    echo "中断します。"
	    exit 0
	else
	    echo $openrtm_repo >> /etc/apt/sources.list
	fi
    fi
}

#----------------------------------------
# パッケージをインストールする
#----------------------------------------
install_packages () {
    for p in $*; do
	if test "x$p" = "x0.4.2" ; then
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
			apt-get install $p
			echo "done." 
			echo ""
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
	rpm -e $p
	echo "done."
	echo ""
    done
}



#---------------------------------------
# メイン
#---------------------------------------
check_root
if test "x$1" = "x-u" ; then
    uninstall_packages `reverse $packages`
else
    update_source_list
    apt-get update
    install_packages $packages
fi

