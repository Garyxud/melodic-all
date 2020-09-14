#!/usr/bin/env perl
#
# @brief CORBA stub and skelton wrapper generator
# @date $Date: 2005-05-12 09:06:18 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2004-2005
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

$date = localtime();
$skel_dir = "rtm/idl/";
$rtc_dir = "rtm/";

#------------------------------------------------------------
# write_skel()
#
# Writing server skeleton wrapper code
#------------------------------------------------------------
sub write_skel {
  ($idl_name) = @_;
  $filename = $idl_name . "Skel.cpp";

  open(SKEL, "> $filename");

  # Header
  print SKEL<<HEADER;
// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * \@file $filename
 * \@brief $idl_name server skeleton wrapper code
 * \@date $date
 *
 */

#include \"${skel_dir}${idl_name}Skel.h\"

HEADER

  # Body
  print SKEL<<BODY;
#if   defined ORB_IS_TAO
#include \"${skel_dir}${idl_name}S.cpp\"
#elif defined ORB_IS_OMNIORB
#include \"${skel_dir}${idl_name}SK.cc\"
#include \"${skel_dir}${idl_name}DynSK.cc\"
#elif defined ORB_IS_MICO
#include \"${skel_dir}${idl_name}_skel.cc\"
#else
#error "NO ORB defined"
#endif

BODY

  close(SKEL);
  print "$filename generated\n";
}




#------------------------------------------------------------
# write_skelh()
#
# Writing server skeleton header wrapper code
#------------------------------------------------------------
sub write_skelh {
  ($idl_name) = @_;
  $filename = $idl_name . "Skel.h";
  $hdefine  = $idl_name;
  $hdefine  =~ tr/[a-z]/[A-Z]/;

  open(SKELH, "> $filename");

  # Header
  print SKELH<<HEADER;
// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * \@file $filename
 * \@brief $idl_name server skeleton header wrapper code
 * \@date $date
 *
 */

#ifndef __${hdefine}SKEL_H__
#define __${hdefine}SKEL_H__

#include <${rtc_dir}config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

HEADER

  # Body
  print SKELH<<BODY;
#if   defined ORB_IS_TAO
#include \"${skel_dir}${idl_name}S.h\"
#elif defined ORB_IS_OMNIORB
#include \"${skel_dir}${idl_name}.hh\"
#elif defined ORB_IS_MICO
#include \"${skel_dir}${idl_name}.h\"
#else
#error "NO ORB defined"
#endif

BODY

  # Footer
  print SKELH<<FOOTER;
#endif // end of __${hdefine}SKEL_H__

FOOTER

  close(SKELH);
  print "$filename generated\n";
}



#------------------------------------------------------------
# write_stub()
#
# Writing stub wrapper code
#------------------------------------------------------------
sub write_stub {
  ($idl_name) = @_;
  $filename = $idl_name . "Stub.cpp";

  open(STUB, "> $filename");

  # Header
  print STUB<<HEADER;
// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * \@file $filename
 * \@brief $idl_name client stub wrapper code
 * \@date $date
 *
 */

#include \"${skel_dir}${idl_name}Stub.h\"

HEADER

  # Body
  print STUB<<BODY;
#if   defined ORB_IS_TAO
#include \"${skel_dir}${idl_name}C.cpp\"
#elif defined ORB_IS_OMNIORB
#include \"${skel_dir}${idl_name}SK.cc\"
#include \"${skel_dir}${idl_name}SynSK.cc\"
#elif defined ORB_IS_MICO
#include \"${skel_dir}${idl_name}.cc\"
#else
#error "NO ORB defined"
#endif

BODY

  close(STUB);
  print "$filename generated\n";
}




#------------------------------------------------------------
# write_stubh()
#
# Writing stub header wrapper code
#------------------------------------------------------------
sub write_stubh {
  ($idl_name) = @_;
  $filename = $idl_name . "Stub.h";
  $hdefine  = $idl_name;
  $hdefine  =~ tr/[a-z]/[A-Z]/;

  open(STUBH, "> $filename");

  # Header
  print STUBH<<HEADER;
// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * \@file $filename
 * \@brief $idl_name client stub header wrapper code
 * \@date $date
 *
 */

#ifndef __${hdefine}STUB_H__
#define __${hdefine}STUB_H__

#include <${rtc_dir}config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

HEADER

  # Body
  print STUBH<<BODY;
#if   defined ORB_IS_TAO
#include \"${skel_dir}${idl_name}C.h\"
#elif defined ORB_IS_OMNIORB
#include \"${skel_dir}${idl_name}.hh\"
#elif defined ORB_IS_MICO
#include \"${skel_dir}${idl_name}.h\"
#else
#error "NO ORB defined"
#endif

BODY

  # Footer
  print STUBH<<FOOTER;
#endif // end of __${hdefine}STUB_H__

FOOTER

  close(STUBH);
  print "$filename generated\n";
}




$idl_name = $ARGV[0];
$idl_name =~ s/\.idl//g;

&write_skel($idl_name);
&write_stub($idl_name);
&write_skelh($idl_name);
&write_stubh($idl_name);



