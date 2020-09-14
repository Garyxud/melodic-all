#!/usr/bin/env python
#
# @brief CORBA stub and skelton wrapper generator
# @date $Date: 2008-02-29 04:50:39 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2005-2011
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

import sys
import os
import re
import time
import yat

skel_cpp_temp = """// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file  [skel_cpp]
 
 * @brief [basename] server skeleton wrapper code
 * @date  [date]
 
 *
 */

#include "[skel_dir]/[basename]Skel.h"

#if   defined ORB_IS_TAO
#include "[skel_dir]/[basename]C.cpp"
#include "[skel_dir]/[basename]S.cpp"
#elif defined ORB_IS_OE
#include "[skel_dir]/[basename].cxx"
#include "[skel_dir]/[basename]_s.cxx"
#elif defined ORB_IS_OMNIORB
#ifdef WIN32
#pragma warning( disable : 4267 )
#pragma warning( disable : 4290 )
#pragma warning( disable : 4311 )
#pragma warning( disable : 4312 )
#endif // WIN32
#include "[skel_dir]/[basename]SK.cc"
#include "[skel_dir]/[basename]DynSK.cc"
#ifdef WIN32
#pragma warning( default : 4267 )
#pragma warning( default : 4290 )
#pragma warning( default : 4311 )
#pragma warning( default : 4312 )
#endif // WIN32
#elif defined ORB_IS_MICO
#include "[skel_dir]/[basename].cc"
#include "[skel_dir]/[basename]_skel.cc"
#elif defined ORB_IS_ORBIT2
#include "[skel_dir]/[basename]-cpp-skels.cc"
#include "[skel_dir]/[basename]-cpp-stubs.cc"
#elif defined ORB_IS_RTORB
#include "[skel_dir]/[basename]-common.c"
#include "[skel_dir]/[basename]-stubs.c"
#include "[skel_dir]/[basename]-skels.c"
#include "[skel_dir]/[basename]-skelimpl.c"
#else
#error "NO ORB defined"
#endif
"""

skel_h_temp = """// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file  [skel_h]
 
 * @brief [basename] server skeleton wrapper code
 * @date  [date]

 *
 */

#ifndef __[BASENAME]SKEL_H__
#define __[BASENAME]SKEL_H__

#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#include "[skel_dir]/[basename]C.h"
#include "[skel_dir]/[basename]S.h"
#elif defined ORB_IS_OE
#include "[skel_dir]/[basename]_s.h"
#include "[skel_dir]/[basename].h"
#elif defined ORB_IS_OMNIORB
#ifdef WIN32
#pragma warning( disable : 4267 )
#pragma warning( disable : 4290 )
#pragma warning( disable : 4311 )
#pragma warning( disable : 4312 )
#endif // WIN32
#include "[skel_dir]/[basename].hh"
#ifdef WIN32
#pragma warning( default : 4267 )
#pragma warning( default : 4290 )
#pragma warning( default : 4311 )
#pragma warning( default : 4312 )
#endif // WIN32
#elif defined ORB_IS_MICO
#include "[skel_dir]/[basename].h"
#elif defined ORB_IS_ORBIT2
#include "[skel_dir]/[basename]-cpp-stubs.h"
#include "[skel_dir]/[basename]-cpp-skels.h"
#elif defined ORB_IS_RTORB
#include "[skel_dir]/[basename].h"
#else
#error "NO ORB defined"
#endif

#endif // end of __[BASENAME]SKEL_H__
"""

stub_cpp_temp = """// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file  [stub_cpp]
 
 * @brief [basename] server skeleton wrapper code
 * @date  [date]
 
 *
 */

#include "[skel_dir]/[basename]Stub.h"

#if   defined ORB_IS_TAO
#include "[skel_dir]/[basename]C.cpp"
#elif defined ORB_IS_OE
#include "[skel_dir]/[basename].cxx"
#elif defined ORB_IS_OMNIORB
#ifdef WIN32
#pragma warning( disable : 4267 )
#pragma warning( disable : 4290 )
#pragma warning( disable : 4311 )
#pragma warning( disable : 4312 )
#endif // WIN32
#include "[skel_dir]/[basename]SK.cc"
#include "[skel_dir]/[basename]DynSK.cc"
#ifdef WIN32
#pragma warning( default : 4267 )
#pragma warning( default : 4290 )
#pragma warning( default : 4311 )
#pragma warning( default : 4312 )
#endif // WIN32
#elif defined ORB_IS_MICO
#include "[skel_dir]/[basename].cc"
#elif defined ORB_IS_ORBIT2
#include "[skel_dir]/[basename]-cpp-stubs.cc"
#elif defined ORB_IS_RTORB
#include "[skel_dir]/[basename]-stubs.c"
#else
#error "NO ORB defined"
#endif
"""

stub_h_temp = """// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file  [stub_h]
 
 * @brief [basename] server skeleton wrapper code
 * @date  [date]

 *
 */

#ifndef __[BASENAME]STUB_H__
#define __[BASENAME]STUB_H__

#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#include "[skel_dir]/[basename]C.h"
#elif defined ORB_IS_OE
#include "[skel_dir]/[basename].h"
#elif defined ORB_IS_OMNIORB
#ifdef WIN32
#pragma warning( disable : 4267 )
#pragma warning( disable : 4290 )
#pragma warning( disable : 4311 )
#pragma warning( disable : 4312 )
#endif // WIN32
#include "[skel_dir]/[basename].hh"
#ifdef WIN32
#pragma warning( default : 4267 )
#pragma warning( default : 4290 )
#pragma warning( default : 4311 )
#pragma warning( default : 4312 )
#endif // WIN32
#elif defined ORB_IS_MICO
#include "[skel_dir]/[basename].h"
#elif defined ORB_IS_ORBIT2
#include "[skel_dir]/[basename]-cpp-stubs.h"
#elif defined ORB_IS_RTORB
#include "[skel_dir]/[basename].h"
#else
#error "NO ORB defined"
#endif

#endif // end of __[BASENAME]STUB_H__
"""

class wrapper_data:
    def __init__(self, basename, dir_name):
        self.data = {}
        self.data["basename"] = basename
        self.data["BASENAME"] = re.sub("-", "_", basename.upper())
        self.data["idl_name"] = basename + ".idl"
        self.data["skel_dir"] = dir_name
        self.data["date"]     = time.ctime()
        self.data["skel_cpp"] = basename + "Skel.cpp"
        self.data["skel_h"]   = basename + "Skel.h"
        self.data["stub_cpp"] = basename + "Stub.cpp"
        self.data["stub_h"]   = basename + "Stub.h"
        
    def get_dict(self):
        return self.data
		

class wrapper_gen:
    def __init__(self, data):
        self.data = data

    def gen(self, fname, temp_txt, data):

        t = yat.Template(temp_txt)
        text = t.generate(data)

        if os.access(fname, os.F_OK): # file exists
            f = file(fname, "r")
            oldtext = f.read()
            f.close()

            newtext = re.sub(" \@date.*?\n", "", text)
            oldtext2 = re.sub(" \@date.*?\n", "", oldtext)
            if newtext == oldtext2:
                print "\"", fname, \
                    "\" already exists and it will be same as new one."
                print "File is not need to be generated."
                return
            else:
                print "\"", fname, "\" already exists but contents are not same"

        f = file(fname, "w")
        f.write(text)
        f.close()
        print "\"", fname, "\"" " was generated."
        return

    def gen_all(self):
        self.write_skel()
        self.write_skelh()
        self.write_stub()
        self.write_stubh()

        import sys
        if sys.platform != 'win32':
            self.omniorb_gcc4_fix()
        return

    def write_skel(self):
        self.gen(self.data["skel_cpp"], skel_cpp_temp, self.data)
        return

    def write_skelh(self):
        self.gen(self.data["skel_h"], skel_h_temp, self.data)
        return

    def write_stub(self):
        self.gen(self.data["stub_cpp"], stub_cpp_temp, self.data)
        return

    def write_stubh(self):
        self.gen(self.data["stub_h"], stub_h_temp, self.data)
        return

    def omniorb_gcc4_fix(self):
        """
        escape the compile error of omniORB's stub/skel on gcc4
        """
        omnistub = self.data["basename"] + ".hh"
        omnistub_tmp = omnistub + ".old"
        omniskelcc =  self.data["basename"] + "SK.cc"
        if os.access(omnistub, os.F_OK) and os.access(omniskelcc, os.F_OK):
            os.rename(omnistub, omnistub_tmp)
            os.system("sed -e \'s/#if defined(__GNUG__) || defined(__DECCXX) /\#if defined(__GNUG__) \&\& (__GNUG__ < 4) ||  defined(__DECCXX) /g\' " + omnistub_tmp + " > " + omnistub)
            os.remove(omnistub_tmp)

idl_file = sys.argv[1]
if len(sys.argv) > 2:
    skel_dir = sys.argv[2]
else:
    skel_dir = "rtm/idl"

basename = os.path.basename(idl_file)
basename = re.sub(".idl", "", basename)
data = wrapper_data(basename, skel_dir)
gen  = wrapper_gen(data.get_dict())
gen.gen_all()
