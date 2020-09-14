#!/usr/bin/env python
#
# @brief WiX wxs file generator for omniORBpy3.4-Python2.6
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
    ("bin/x86_win32",                                             "*.dll *.exe"),
    ("bin/dll",                                                   "*.dll"),
    ("lib/x86_win32",                                             "*.pyd"),
    ("lib/python/",                                               "*.py"),
    ("lib/python/CosNaming",                                      "*.py"),
    ("lib/python/CosNaming__POA",                                 "*.py"),
    ("lib/python/omniidl",                                        "*.py"),
    ("lib/python/omniidl_be",                                     "*.py"),
    ("lib/python/omniORB",                                        "*.py"),
    ("lib/python/omniORB/COS",                                    "*.py *.pth"),
    ("lib/python/omniORB/COS/CosCollection",                      "*.py"),
    ("lib/python/omniORB/COS/CosCollection__POA",                 "*.py"),
    ("lib/python/omniORB/COS/CosCompoundLifeCycle",               "*.py"),
    ("lib/python/omniORB/COS/CosCompoundLifeCycle__POA",          "*.py"),
    ("lib/python/omniORB/COS/CosContainment",                     "*.py"),
    ("lib/python/omniORB/COS/CosContainment__POA",                "*.py"),
    ("lib/python/omniORB/COS/CosEventChannelAdmin",               "*.py"),
    ("lib/python/omniORB/COS/CosEventChannelAdmin__POA",          "*.py"),
    ("lib/python/omniORB/COS/CosEventComm",                       "*.py"),
    ("lib/python/omniORB/COS/CosEventComm__POA",                  "*.py"),
    ("lib/python/omniORB/COS/CosExternalization",                 "*.py"),
    ("lib/python/omniORB/COS/CosExternalizationContainment",      "*.py"),
    ("lib/python/omniORB/COS/CosExternalizationContainment__POA", "*.py"),
    ("lib/python/omniORB/COS/CosExternalizationReference",        "*.py"),
    ("lib/python/omniORB/COS/CosExternalizationReference__POA",   "*.py"),
    ("lib/python/omniORB/COS/CosExternalization__POA",            "*.py"),
    ("lib/python/omniORB/COS/CosGraphs",                          "*.py"),
    ("lib/python/omniORB/COS/CosGraphs__POA",                     "*.py"),
    ("lib/python/omniORB/COS/CosLifeCycle",                       "*.py"),
    ("lib/python/omniORB/COS/CosLifeCycleContainment",            "*.py"),
    ("lib/python/omniORB/COS/CosLifeCycleContainment__POA",       "*.py"),
    ("lib/python/omniORB/COS/CosLifeCycleReference",              "*.py"),
    ("lib/python/omniORB/COS/CosLifeCycleReference__POA",         "*.py"),
    ("lib/python/omniORB/COS/CosLifeCycle__POA",                  "*.py"),
    ("lib/python/omniORB/COS/CosNaming",                          "*.py"),
    ("lib/python/omniORB/COS/CosNaming__POA",                     "*.py"),
    ("lib/python/omniORB/COS/CosNotification",                    "*.py"),
    ("lib/python/omniORB/COS/CosNotification__POA",               "*.py"),
    ("lib/python/omniORB/COS/CosNotifyChannelAdmin",              "*.py"),
    ("lib/python/omniORB/COS/CosNotifyChannelAdmin__POA",         "*.py"),
    ("lib/python/omniORB/COS/CosNotifyComm",                      "*.py"),
    ("lib/python/omniORB/COS/CosNotifyComm__POA",                 "*.py"),
    ("lib/python/omniORB/COS/CosNotifyFilter",                    "*.py"),
    ("lib/python/omniORB/COS/CosNotifyFilter__POA",               "*.py"),
    ("lib/python/omniORB/COS/CosObjectIdentity",                  "*.py"),
    ("lib/python/omniORB/COS/CosObjectIdentity__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosPersistenceDDO",                  "*.py"),
    ("lib/python/omniORB/COS/CosPersistenceDDO__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosPersistenceDS_CLI",               "*.py"),
    ("lib/python/omniORB/COS/CosPersistenceDS_CLI__POA",          "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePDS",                  "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePDS_DA",               "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePDS_DA__POA",          "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePDS__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePID",                  "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePID__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePO",                   "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePOM",                  "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePOM__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosPersistencePO__POA",              "*.py"),
    ("lib/python/omniORB/COS/CosPropertyService",                 "*.py"),
    ("lib/python/omniORB/COS/CosPropertyService__POA",            "*.py"),
    ("lib/python/omniORB/COS/CosQuery",                           "*.py"),
    ("lib/python/omniORB/COS/CosQueryCollection",                 "*.py"),
    ("lib/python/omniORB/COS/CosQueryCollection__POA",            "*.py"),
    ("lib/python/omniORB/COS/CosQuery__POA",                      "*.py"),
    ("lib/python/omniORB/COS/CosReference",                       "*.py"),
    ("lib/python/omniORB/COS/CosReference__POA",                  "*.py"),
    ("lib/python/omniORB/COS/CosRelationships",                   "*.py"),
    ("lib/python/omniORB/COS/CosRelationships__POA",              "*.py"),
    ("lib/python/omniORB/COS/CosStream",                          "*.py"),
    ("lib/python/omniORB/COS/CosStream__POA",                     "*.py"),
    ("lib/python/omniORB/COS/CosTime",                            "*.py"),
    ("lib/python/omniORB/COS/CosTimerEvent",                      "*.py"),
    ("lib/python/omniORB/COS/CosTimerEvent__POA",                 "*.py"),
    ("lib/python/omniORB/COS/CosTime__POA",                       "*.py"),
    ("lib/python/omniORB/COS/CosTrading",                         "*.py"),
    ("lib/python/omniORB/COS/CosTradingDynamic",                  "*.py"),
    ("lib/python/omniORB/COS/CosTradingDynamic__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosTradingRepos",                    "*.py"),
    ("lib/python/omniORB/COS/CosTradingRepos__POA",               "*.py"),
    ("lib/python/omniORB/COS/CosTrading__POA",                    "*.py"),
    ("lib/python/omniORB/COS/CosTypedEventChannelAdmin",          "*.py"),
    ("lib/python/omniORB/COS/CosTypedEventChannelAdmin__POA",     "*.py"),
    ("lib/python/omniORB/COS/CosTypedEventComm",                  "*.py"),
    ("lib/python/omniORB/COS/CosTypedEventComm__POA",             "*.py"),
    ("lib/python/omniORB/COS/CosTypedNotifyChannelAdmin",         "*.py"),
    ("lib/python/omniORB/COS/CosTypedNotifyChannelAdmin__POA",    "*.py"),
    ("lib/python/omniORB/COS/CosTypedNotifyComm",                 "*.py"),
    ("lib/python/omniORB/COS/CosTypedNotifyComm__POA",            "*.py"),
    ("lib/python/omniORB/COS/LifeCycleService",                   "*.py"),
    ("lib/python/omniORB/COS/LifeCycleService__POA",              "*.py"),
    ("lib/python/omniORB/COS/RDITestTypes",                       "*.py"),
    ("lib/python/omniORB/COS/RDITestTypes__POA",                  "*.py"),
    ("lib/python/omniORB/COS/TimeBase",                           "*.py"),
    ("lib/python/omniORB/COS/TimeBase__POA",                      "*.py")
]


## Resource path
##
base_dir = os.getenv("OMNIORB_PY26")
if base_dir == None:
    base_dir = "C:\\distribution\\omniORBpy-3.4-Python2.6\\"
else:
    base_dir = base_dir.replace("\"", "")
    base_dir += "\\"


## make temporary files
##
temp_dir = base_dir + "bin\\dll"
dll_list = glob.glob(base_dir + "bin\\x86_win32\\*.dll")
dll_cnt = len(dll_list)
if os.path.exists(temp_dir) :
    shutil.rmtree(temp_dir)
os.mkdir(temp_dir)
for i in range(dll_cnt):
    shutil.copy2(dll_list[i], temp_dir)


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
    comp_name = path_to_comp_id(path, "py26")
    # wxs directory name
    dir_name = path_to_dir_id(path, "py26")

    path = path.replace("/", "\\")

    # full path to target directory
    full_path = base_dir + "\\" + path

    flist = []
    for f in files.split(" "):
        flist += glob.glob(full_path + "\\" + f)

    cmd = ["flist",
           "-c", comp_name,
           "-o", dir_name + ".yaml",
           "-p",  base_dir + path]
    cmd += flist

    makewxs.main(cmd)


## make wxs file
##
cmd = ["wxs",
       "-o", "omniORBpy26_inc.wxs",
       "-i", "omniORBpy26_inc.wxs.in"]
cmd += glob.glob("*.yaml")
makewxs.main(cmd)

