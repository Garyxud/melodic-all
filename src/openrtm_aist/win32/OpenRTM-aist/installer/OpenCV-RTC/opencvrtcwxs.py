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

data = [
    ("bin",             "*.dll *.exe rtc.conf")
]

import os
base_dir = os.getenv("OPENCV_RTC_ROOT")
base_dir = base_dir.replace("\"", "")

if base_dir == None:
    base_dir="C:\distribution\ImageProcessing\opencv"
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
    output = prefix
    for c in path.split("/"):
        output += c.capitalize()
    return output.replace(".", "_")

import makewxs

for (path, files) in data:
    # wxs component name
    comp_name = path_to_comp_id(path, "OpenCVRTC")
    # wxs directory name
    dir_name = path_to_dir_id(path, "OpenCVRTC")

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
       "-o", "OpenCV-RTC_inc.wxs",
       "-i", "OpenCV-RTC_inc.wxs.in"]
cmd += glob.glob("*.yaml")
makewxs.main(cmd)
