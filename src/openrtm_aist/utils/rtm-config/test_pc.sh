#!/bin/sh

pkg_config_test()
{
echo "------------------------------------------------------------"
echo "  cmd:" $1
echo "------------------------------------------------------------"
echo "Prefix      : " `$cmd --variable=prefix`
echo "Exec Prefix : " `$cmd --variable=exec_prefix`
echo "Data Prefix : " `$cmd --variable=data_prefix`
echo "Version     : " `$cmd --modversion`
echo "CXX         : " `$cmd --variable=rtm_cxx`
echo "Cflags      : " `$cmd --cflags`
echo "Cflags -I   : " `$cmd --cflags-only-I`
echo "Cflags other: " `$cmd --cflags-only-other`
echo "Libs        : " `$cmd --libs`
echo "Libs static : " `$cmd --static`
echo "Libs -L     : " `$cmd --libs-only-L`
echo "Libs -l     : " `$cmd --libs-only-l`
echo "Libs other  : " `$cmd --libs-only-other`
echo "Lib Dir     : " `$cmd --variable=libdir`
echo "ORB         : " `$cmd --variable=rtm_orb`
echo "IDLC        : " `$cmd --variable=rtm_idlc`
echo "IDLFLAGS    : " `$cmd --variable=rtm_idlflags`
echo "RTM inc dir : " `$cmd --variable=rtm_includedir`
echo "RTM IDL dir : " `$cmd --variable=rtm_idldir`
echo "RTM lib dir : " `$cmd --variable=rtm_libdir`
echo "RTM data dir: " `$cmd --variable=rtm_datadir`
echo "RTM rtcd dir: " `$cmd --variable=rtm_rtcdir`
echo "RTM EC dir  : " `$cmd --variable=rtm_ecdir`
echo "RTM rtm dir : " `$cmd --variable=rtm_rtmdir`
echo "RTM svc dir : " `$cmd --variable=rtm_svcdir`
echo "RTM doc dir : " `$cmd --variable=rtm_docdir`
echo "RTM exp dir : " `$cmd --variable=rtm_exampledir`
echo "coil inc dir: " `$cmd --variable=coil_includedir`
}
export PKG_CONFIG_PATH=./
cmd="pkg-config openrtm-aist"
pkg_config_test $cmd

echo "------------------------------------------------------------"
