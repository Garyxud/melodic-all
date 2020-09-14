#!/usr/bin/env python
#
# @brief WiX wxs file generator for OpenRTM-aist-Python
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

import os
import shutil
import glob
import makewxs

data = [
    ("docs/ClassReference-jp/html",               "*.css *.gif *.png *.html *.hhc *.hhk *.hhp"),
    ("docs/ClassReference-en/html",               "*.css *.gif *.png *.html *.hhc *.hhk *.hhp"),
    ("idlcompiler",                                 "idlcompile.bat idlcompile.py"),
    ("OpenRTM_aist/utils/rtm-naming",             "rtm-naming.py"),
    ("OpenRTM_aist/examples",                     "*.conf"),
    ("OpenRTM_aist/examples/AutoControl",         "*.py *.conf"),
    ("OpenRTM_aist/examples/Composite",           "*.py *.conf"),
    ("OpenRTM_aist/examples/ConfigSample",        "*.py *.conf"),
    ("OpenRTM_aist/examples/ExtTrigger",          "*.py *.conf"),
    ("OpenRTM_aist/examples/MobileRobotCanvas",   "*.py *.conf"),
    ("OpenRTM_aist/examples/NXTRTC",              "*.py *.conf"),
    ("OpenRTM_aist/examples/SeqIO",               "*.py *.conf"),
    ("OpenRTM_aist/examples/SimpleIO",            "*.py *.conf"),
    ("OpenRTM_aist/examples/SimpleService",       "*.py *.conf *.idl"),
    ("OpenRTM_aist/examples/Slider_and_Motor",    "*.py *.conf"),
    ("OpenRTM_aist/examples/TkJoyStick",          "*.py *.conf"),
    ("OpenRTM_aist/examples/TkLRFViewer",         "*.py *.conf"),
    ("OpenRTM_aist/examples/Templates",           "*.py *.xml"),
    ("OpenRTM_aist24",                            "*.py"),
    ("OpenRTM_aist24/ext",                        "*.py"),
    ("OpenRTM_aist24/ext/sdo",                    "*.py"),
    ("OpenRTM_aist24/ext/sdo/observer",           "*.py *.idl rtc.conf setup.bat"),
    ("OpenRTM_aist24/utils/rtcd",                 "rtcd.py rtcd_python.bat rtcd_python.exe rtcd.conf"),
    ("OpenRTM_aist24/utils/rtcprof",              "rtcprof.py rtcprof_python.bat"),
    ("OpenRTM_aist24/utils/rtc-template",         "*.py"),
    ("OpenRTM_aist24/RTM_IDL",                    "*.py *.idl *.pth"),
    ("OpenRTM_aist24/RTM_IDL/device_interfaces",  "*.py *.idl"),
    ("OpenRTM_aist24/root",                       "*.pth"),
    ("OpenRTM_aist25",                            "*.py"),
    ("OpenRTM_aist25/ext",                        "*.py"),
    ("OpenRTM_aist25/ext/sdo",                    "*.py"),
    ("OpenRTM_aist25/ext/sdo/observer",           "*.py *.idl rtc.conf setup.bat"),
    ("OpenRTM_aist25/utils/rtcd",                 "rtcd.py rtcd_python.bat rtcd_python.exe rtcd.conf"),
    ("OpenRTM_aist25/utils/rtcprof",              "rtcprof.py rtcprof_python.bat"),
    ("OpenRTM_aist25/utils/rtc-template",         "*.py"),
    ("OpenRTM_aist25/RTM_IDL",                    "*.py *.idl *.pth"),
    ("OpenRTM_aist25/RTM_IDL/device_interfaces",  "*.py *.idl"),
    ("OpenRTM_aist25/root",                       "*.pth"),
    ("OpenRTM_aist26",                            "*.py"),
    ("OpenRTM_aist26/ext",                        "*.py"),
    ("OpenRTM_aist26/ext/sdo",                    "*.py"),
    ("OpenRTM_aist26/ext/sdo/observer",           "*.py *.idl rtc.conf setup.bat"),
    ("OpenRTM_aist26/utils/rtcd",                 "rtcd.py rtcd_python.bat rtcd_python.exe rtcd.conf"),
    ("OpenRTM_aist26/utils/rtcprof",              "rtcprof.py rtcprof_python.bat"),
    ("OpenRTM_aist26/utils/rtc-template",         "*.py"),
    ("OpenRTM_aist26/RTM_IDL",                    "*.py *.idl *.pth"),
    ("OpenRTM_aist26/RTM_IDL/device_interfaces",  "*.py *.idl"),
    ("OpenRTM_aist26/root",                       "*.pth")
]

## Resource path
##
base_dir = os.getenv("OPENRTM_PY")
if base_dir == None:
    base_dir = "C:\\distribution\\OpenRTM-aist-Python-1.1.0\\"
else:
    base_dir = base_dir.replace("\"", "")
    base_dir += "\\"

## make temporary files
##
src_dir = base_dir + "OpenRTM_aist"
temp_dir24 = base_dir + "OpenRTM_aist24"
temp_dir25 = base_dir + "OpenRTM_aist25"
temp_dir26 = base_dir + "OpenRTM_aist26"
dll_list = glob.glob(base_dir + "bin\\x86_win32\\*.dll")
dll_cnt = len(dll_list)
if os.path.exists(temp_dir24) :
    shutil.rmtree(temp_dir24)
if os.path.exists(temp_dir25) :
    shutil.rmtree(temp_dir25)
if os.path.exists(temp_dir26) :
    shutil.rmtree(temp_dir26)
shutil.copytree(src_dir, temp_dir24)
shutil.copytree(src_dir, temp_dir25)
shutil.copytree(src_dir, temp_dir26)

temp_dir24_root = base_dir + "OpenRTM_aist24\\root"
temp_dir25_root = base_dir + "OpenRTM_aist25\\root"
temp_dir26_root = base_dir + "OpenRTM_aist26\\root"
os.mkdir(temp_dir24_root)
os.mkdir(temp_dir25_root)
os.mkdir(temp_dir26_root)
shutil.copy2(base_dir + "OpenRTM-aist.pth", temp_dir24_root)
shutil.copy2(base_dir + "OpenRTM-aist.pth", temp_dir25_root)
shutil.copy2(base_dir + "OpenRTM-aist.pth", temp_dir26_root)


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


## make yaml file
##
for (path, files) in data:
    # wxs component name
    comp_name = path_to_comp_id(path, "rtm")
    comp_name = comp_name.replace("-", "")
    # wxs directory name
    dir_name = path_to_dir_id(path, "rtm")

    path = path.replace("/", "\\")

    # full path to target directory
    full_path = base_dir + "\\" + path

    flist = []
    for f in files.split(" "):
        flist += glob.glob(full_path + "\\" + f)

    curr_path = base_dir + path
    if curr_path.rfind("\\") == (len(curr_path) -1):
        curr_path = curr_path[0:len(curr_path)-1]

    cmd = ["flist",
           "-c", comp_name,
           "-o", dir_name + ".yaml",
           "-p", curr_path]
    cmd += flist
    makewxs.main(cmd)


## make wxs file
##
cmd = ["wxs",
       "-o", "OpenRTM-aist-Python.wxs",
       "-i", "OpenRTM-aist-Python.wxs.in"]
cmd += glob.glob("*.yaml")
makewxs.main(cmd)

