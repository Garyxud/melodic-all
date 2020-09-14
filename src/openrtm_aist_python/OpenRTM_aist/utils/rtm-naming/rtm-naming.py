#!/usr/bin/env python
# -*- Python -*-
#
#  @file rtm-naming.py
#  @brief OpenRTM-aist name server launcher
#  @date $Date: 2007/10/25 $
#  @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
#  Copyright (C) 2003-2009
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 

default_orb="omniORB"
orb=default_orb
default_port="2809"


import sys,os,platform


def get_hostname():
    sysinfo  = platform.uname()
    hostname = sysinfo[1]
    return str(hostname)


def get_tempdir():
    temp_dir = os.environ.get('TEMP', "")
    if temp_dir != "":
        return temp_dir

    temp_dir = os.environ.get('TMP', "")
    if temp_dir != "":
        return temp_dir

    return os.getcwd()


def find_nscmd(ns_cmd, ns_env = ""):
    if sys.platform == "win32":
        ns_path = os.path.join(sys.exec_prefix,ns_cmd)
    else:
        ns_path = os.path.join(sys.exec_prefix,'bin',ns_cmd)

    if os.path.exists(ns_path):
        return ns_path

    if ns_env != "":
        ns_path = os.environ.get(ns_env) + ns_cmd
        if os.path.exists(ns_path):
            return ns_path

    sys_paths = os.environ.get('PATH').split(";")
    for sys_path in sys_paths:
        ns_path = sys_path + ns_cmd
        if os.path.exists(ns_path):
            return ns_path
    return None


def del_file(file_path):
    if sys.platform == "win32":
        delcmd = "del /F "
    else:
        delcmd = "rm -f "
    delcmd += file_path
    return os.system(delcmd)


def usage():
    print "Usage: python rtm-naming.py port_number"


def omninames(port = "", endpoint = ""):
    hostname = get_hostname()
    log_path = get_tempdir()

    log_fname = "omninames-" + hostname + ".log"
    log_file = os.path.join(log_path, log_fname)

    if os.path.exists(log_file):
        del_file(log_file)

    bak_fname = "omninames-" + hostname + ".bak"
    bak_file = os.path.join(log_path, bak_fname)

    if os.path.exists(bak_file):
        del_file(bak_file)

    if port == "":
        port = default_port

    print "Starting omniORB omniNames: ", hostname, ":", port

    if sys.platform == "win32":
        omniNames = find_nscmd("omniNames.exe", "OMNI_ROOT")
    else:
        omniNames = find_nscmd("omniNames")

    if not omniNames:
        print "Not found omniNames."
        sys.exit()

    cmd  = omniNames 
    cmd += " -start " + str(port)
    cmd += " -logdir \"" + str(log_path) + "\""
    print cmd
    os.system(cmd)


if __name__ == "__main__":
    try:
        if len(sys.argv) == 2:
            port = int(sys.argv[1])

            if sys.argv[1] == "-u" or sys.argv[1] == "-h" or sys.argv[1] == "--help":
                usage()
    except:
        usage()
        sys.exit(1)
        
    if orb == "omniORB":
        omninames()
