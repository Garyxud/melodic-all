#!/usr/bin/env python
#
# @brief WiX wxs file generator for omniORB
# @date $Date$
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2010
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

omni_files = "COPYING CREDITS THIS_IS_OMNIORB_4_1_4 COPYING.LIB update.log COPYING.PYTHON sample.reg README.FIRST.txt README.win32.txt ReleaseNotes.txt"

data = [
    ("",                                 omni_files),
    ("bin/scripts",                       "*.py"),
    ("bin/x86_win32",                     "*.dll *.exe"),
    ("idl",                               "*.idl"),
    ("idl/COS",                           "*.idl"),
    ("include",                           "*.h"),
    ("include/COS",                       "*.h *.hh"),
    ("include/omniORB4",                  "*.h *.hh"),
    ("include/omniORB4/internal",         "*.h *.hh"),
    ("include/omnithread",                "*.h"),
    ("include/omniVms",                   "*.hxx"),
    ("include/python1.5",                 "*.h"),
    ("lib/python/omniidl",                "*.py"),
    ("lib/python/omniidl_be",             "*.py"),
    ("lib/python/omniidl_be/cxx",         "*.py"),
    ("lib/python/omniidl_be/cxx/dynskel", "*.py"),
    ("lib/python/omniidl_be/cxx/header",  "*.py"),
    ("lib/python/omniidl_be/cxx/impl",    "*.py"),
    ("lib/python/omniidl_be/cxx/skel",    "*.py"),
    ("lib/python1.5",                     "*.py*"),
    ("lib/x86_win32",                     "*.lib"),
    ("src",                               "GNUmakefile")
]

import os
base_dir = os.getenv("OMNI_ROOT")
base_dir = base_dir.replace("\"", "")

if base_dir == None:
    base_dir="C:\\Program Files\\omniORB-4.1.4\\"
else:
    base_dir += "\\"

def path_to_dir_id(path, prefix):
    # path = "bin/x86_win32" prefix = "omni"
    # output = "omni_bin_x86_win32"
    # "." -> "_"
    output = prefix + "_" + "_".join(path.split("/"))
    return output.replace(".", "_")


def path_to_comp_id(path, prefix):
    # path = "bin/x86_win32" prefix = "omni"
    # output = "OmniBinX86_win32"
    # "." -> "_"
    output = prefix.capitalize()
    for c in path.split("/"):
        output += c.capitalize()
    return output.replace(".", "_")

import makewxs

for (path, files) in data:
    # wxs component name
    comp_name = path_to_comp_id(path, "omni")
    # wxs directory name
    dir_name = path_to_dir_id(path, "omni")

    path = path.replace("/", "\\")

    # full path to target directory
    full_path = base_dir + "\\\\" + path

    import glob
    flist = []
    for f in files.split(" "):
        flist += glob.glob(full_path + "\\" + f)

    cmd = ["flist",
           "-c", comp_name,
           "-o", dir_name + ".yaml",
           "-p",  base_dir + path]
    cmd += flist

    makewxs.main(cmd)


cmd = ["wxs",
       "-o", "omniORB_inc.wxs",
       "-i", "omniORB_inc.wxs.in"]
cmd += glob.glob("*.yaml")
makewxs.main(cmd)
