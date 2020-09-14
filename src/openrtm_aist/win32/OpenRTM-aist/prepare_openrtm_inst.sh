#!/bin/sh
#
# @brief Preparing omniORB WiX merge module build environment
# @author Noriaki Ando <n-ando@aist.go.jp>
#         Copyright 2014 (C) All Right Reserved
#
# This script copy the following omniORB files to specific directories
#   runtime: DLL and EXE for runtime environemnt
#      --> omniORB_runtime
#   devel  : headers, omniidl and backends python scripts and others
#      --> omniORB_devel
#
# OpenRTM_runtime
#   + <version>
#     + bin
#     + etc
#     + ext
#       + sdo
#
# OpenRTM_devel
#   + <version>
#     + cmake
#     + coil
#     + lib
#     + rtm
#       + idl
#     + utils
#       + rtc-template
#
# OpenRTM_examples
#   + <version>
#     + examples
#       + C++
#
# OpenRTM_doc
#   + <version>
#     + doc
#

export PATH=/cygdrive/c/cygwin64/bin:$PATH
echo `pwd`
VERSION=""
#ARCH=""
#VC_VERSION=""

script_name=$(basename $0)

usage()
{
    echo <<EOF
    $script_name <filename or URL>

Local file mode:
 $script_name OpenRTM-bin.zip

Download mode
 $script_name http://<OpenRTM bin URL>/OpenRTM-X.Y.Z-winXX-vcX.zip
 
EOF
}

download_openrtm_binpkg()
{
    wget $DOWNLOAD_URL
}

find_openrtm_binpkg()
{
    VERSION=`echo $LOCAL_FILE | sed 's/OpenRTM-aist-\([0-9\.]*\)-.*.zip/\1/'`
    ARCH=`echo $LOCAL_FILE | sed 's/OpenRTM-.*-win\([0-9]*\)-.*.zip/\1/'`
    VC_VERSION=`echo $LOCAL_FILE | sed 's/OpenRTM.*-vc\([0-9]*\).*.zip/\1/'`
    OPENRTM_PKG=$LOCAL_FILE
    echo "VERSION: " $VERSION
    RUNTIME_DIR=OpenRTM_runtime/${VERSION}_vc$VC_VERSION
    DEVEL_DIR=OpenRTM_devel/${VERSION}_vc$VC_VERSION
    EXAMPLE_DIR=OpenRTM_example/${VERSION}_vc$VC_VERSION
    DOC_DIR=OpenRTM_doc/${VERSION}_vc$VC_VERSION
}

extract_package()
{
    find -maxdepth 1 -type d -name 'OpenRTM-*' -exec rm -rf {} \;
    unzip -x $OPENRTM_PKG
    OPENRTM_DIR=`find -maxdepth 1 -type d -name 'OpenRTM-*'`
}

cleanup_dirs()
{
    OPENRTM_DIR=`find -maxdepth 1 -type d -name 'OpenRTM-*'`
    rm -rf OpenRTM_runtime
    rm -rf OpenRTM_examples
    rm -rf OpenRTM_devel
    rm -rf OpenRTM_doc
}

create_dirs()
{
    mkdir -p $RUNTIME_DIR/{bin,etc,ext/sdo}
    mkdir -p $EXAMPLE_DIR//examples/C++
    mkdir -p $DEVEL_DIR/{cmake,coil,lib,rtm/idl,utils/rtc-template}
    mkdir -p $DOC_DIR/doc/C++
    find . -type d -exec chmod 755 {} \;
    find . -type f -exec chmod 644 {} \;
}

copy_runtime_files()
{
    if test "x$OPENRTM_DIR" = "x" ; then
        echo "Variable OPENRTM_DIR is not set. Aborting"
        exit -1
    fi
    cp $OPENRTM_DIR/AUTHORS $RUNTIME_DIR/
    cp $OPENRTM_DIR/ChangeLog $RUNTIME_DIR/
    cp $OPENRTM_DIR/COPYING.LIB $RUNTIME_DIR/
    cp $OPENRTM_DIR/COPYRIGHT $RUNTIME_DIR/
    cp $OPENRTM_DIR/INSTALL.jp $RUNTIME_DIR/
    cp $OPENRTM_DIR/README $RUNTIME_DIR/
    cp $OPENRTM_DIR/README.jp $RUNTIME_DIR/
    cp $OPENRTM_DIR/THIS_IS_OPENRTM* $RUNTIME_DIR/
    
    cp $OPENRTM_DIR/bin/*.dll    $RUNTIME_DIR/bin/
    cp $OPENRTM_DIR/bin/*.exe    $RUNTIME_DIR/bin/
    cp $OPENRTM_DIR/bin/*.py     $RUNTIME_DIR/bin/
    cp $OPENRTM_DIR/bin/*.bat    $RUNTIME_DIR/bin/
    cp $OPENRTM_DIR/bin/rtc.conf $RUNTIME_DIR/bin/
    cp RTM_ROOT_dir.wxsctrl      $RUNTIME_DIR/
    cp OpenRTM_bin_dir.wxsctrl   $RUNTIME_DIR/bin/

    cp $OPENRTM_DIR/etc/rtc.conf.sample $RUNTIME_DIR/etc/

    cp $OPENRTM_DIR/components/ComponentObserverConsumer.dll $RUNTIME_DIR/ext/sdo
}

copy_devel_files()
{
    if test "x$OPENRTM_DIR" = "x" ; then
        echo "Variable OPENRTM_DIR is not set. Aborting"
        exit -1
    fi
    cp -r $OPENRTM_DIR/cmake       $DEVEL_DIR
    cp $OPENRTM_DIR/coil/*.h    $DEVEL_DIR/coil
    cp $OPENRTM_DIR/coil/*.cpp  $DEVEL_DIR/coil
    cp $OPENRTM_DIR/bin/*.lib   $DEVEL_DIR/lib
    cp $OPENRTM_DIR/rtm/*.h     $DEVEL_DIR/rtm
    cp $OPENRTM_DIR/rtm/*.cpp   $DEVEL_DIR/rtm
    cp $OPENRTM_DIR/rtm/*.txt   $DEVEL_DIR/rtm
    cp $OPENRTM_DIR/rtm/idl/*.idl $DEVEL_DIR/rtm/idl
    cp $OPENRTM_DIR/rtm/idl/*.cc $DEVEL_DIR/rtm/idl
    cp $OPENRTM_DIR/rtm/idl/*.cpp $DEVEL_DIR/rtm/idl
    cp $OPENRTM_DIR/rtm/idl/*.h $DEVEL_DIR/rtm/idl
    cp $OPENRTM_DIR/rtm/idl/*.hh $DEVEL_DIR/rtm/idl
    cp -r $OPENRTM_DIR/utils/rtc-template  $DEVEL_DIR/utils/
}

copy_example_files()
{
    if test "x$OPENRTM_DIR" = "x" ; then
        echo "Variable OPENRTM_DIR is not set. Aborting"
        exit -1
    fi
    ORG_EX=$OPENRTM_DIR/examples/
    DST_EX=$EXAMPLE_DIR/examples/C++
    cp $OPENRTM_DIR/components/*.dll  $EXAMPLE_DIR/examples/C++
    rm -f $EXAMPLEL_DIR/examples/C++/ComponentObserverConsumer.dll
    cp $OPENRTM_DIR/components/*.exe  $EXAMPLE_DIR/examples/C++

    cp $ORG_EX/SimpleIO/rtc.conf              $DST_EX/
    cp $ORG_EX/Composite/rtc_win32.conf       $DST_EX/rtc_composite.conf
    cp $ORG_EX/Composite/composite.conf       $DST_EX/
    cp $ORG_EX/ConfigSample/configsample.conf $DST_EX/
}

copy_doc_files()
{
    if test "x$OPENRTM_DIR" = "x" ; then
        echo "Variable OPENRTM_DIR is not set. Aborting"
        exit -1
    fi
    cp -r $OPENRTM_DIR/docs/*  $DOC_DIR/doc/C++
}

#==============================
# main
#==============================

if test $# != 1; then
    usage
    exit -1
fi
arg1=`echo $1 | grep '^http'`
if test "x$arg1" != "x"; then
    DOWNLOAD_URL=$1
    download_omniorb_binpkg
    LOCAL_FILE=`basename $DOWNLOAD_URL`
else
    LOCAL_FILE=$1
fi
if test ! -f $LOCAL_FILE ; then
    echo "No such file: " $LOCAL_FILE
    echo "Aborting."
    exit -1
fi

find_openrtm_binpkg
extract_package
cleanup_dirs
create_dirs
copy_runtime_files
copy_devel_files
copy_example_files
copy_doc_files

# end of ecript
#==============================
