#!/bin/sh
#
# @brief Preparing OpenRTM-aist build environment on Windows
# @author Noriaki Ando <n-ando@aist.go.jp>
#         Copyright 2014 (C) All Right Reserved
#
# This is part of build.bat omniORB build batch file
# for building omniORB for Windows binaries.
#
# The following environment variables should be set.
#
if test "x$OMNI_VERSION" = "x" ; then
    export OMNI_VERSION=4.2.0
fi
if test "x$OMNITH_VER" = "x" ; then
    export OMNITH_VER=4.0
fi
if test "x$PYTHON_DIR" = "x" ; then
    export PYTHON_DIR=/cygdrive/c/Python27
fi
if test "x$VC_VERSION" = "x" ; then
    export VC_VERSION=12
fi
if test "x$ARCH" = "x" ; then
    export ARCH=x86
fi

export PATH=/cygdrive/c/cygwin64/bin:$PATH
echo `pwd`

check_env()
{

    if test "x$ARCH" = "xx86" ;then
        WIN_ARCH="win32"
    elif test "x$ARCH" = "xx86_64" ;then
        WIN_ARCH="win64"
    else
        echo "ARCH not defined"
        exit -1
    fi
    if test "x$VC_VERSION" = "x" ; then
        echo "VC_VERSION not defined."
        exit -1
    fi
    if test "x$OMNI_VERSION" = "x" ; then
        echo "OMNI_VERSION not defined."
        exit -1
    else
        OMNI_SHORT_VER=`echo $OMNI_VERSION | sed 's/\.//g'`
    fi
    if test "x$OMNITH_VER" = "x" ; then
        echo "OMNITH_VER not defined."
        exit -1
    else
        OMNITH_SHORT_VER=`echo $OMNITH_VER | sed 's/\.//g'`
    fi
    if test "xPYTHON_DIR" = "x" ; then
        echo "PYTHON_DIR not defined."
        exit -1
    else
        PY_VERSION=`echo ${PYTHON_DIR} | sed 's/.*ython\([0-9][0-9]\).*/\1/'`
    fi
    base_url="http://openrtm.org/pub/omniORB/win32/omniORB-${OMNI_VERSION}/"

    OMNIORB_ZIP=omniORB-${OMNI_VERSION}-${WIN_ARCH}-vc${VC_VERSION}-py${PY_VERSION}.zip
    OMNIORB_URL=${base_url}/${OMNIORB_ZIP}
}

unpack_omnibin()
{
    omnidir=`find ./ -maxdepth 1 -name 'omniORB*' -type d`
    if test -d $omnidir ; then
        rm -rf $omnidir
    fi
    # getting omniORB-4.1.7.tar.bz2
    if test ! -f $OMNIORB_ZIP ; then
        wget $OMNIORB_URL
    fi
    unzip $OMNIORB_ZIP
}

rename_omnidir()
{
    omnidir=`find ./ -maxdepth 1 -name 'omniORB*' -type d`
    mv ${omnidir} omniORB
}
check_env
unpack_omnibin
rename_omnidir

# end of ecript
#==============================
