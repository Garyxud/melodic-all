#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# \file Process.py
# \brief Process handling functions
# \author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2010
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import os,sys
import traceback
import subprocess

##
# @if jp
# @brief プロセスを起動する
# @else
# @brief Launching a process
# @endif
#
# int launch_shell(std::string command)
def launch_shell(command):
  args = command.split(" ")

  if sys.platform == "win32":
    CREATE_NEW_PROCESS_GROUP = 0x00000200
    subproc_args = { 'stdin':     None,
                     'stdout':    None,
                     'stderr':    None,
                     'cwd':       None,
                     'close_fds': False,
                     'creationflags': CREATE_NEW_PROCESS_GROUP}
  else:
    subproc_args = { 'stdin':     None,
                     'stdout':    None,
                     'stderr':    None,
                     'cwd':       None,
                     'close_fds': False,
                     'preexec_fn': os.setsid}

  try:
    p = subprocess.Popen(args, **subproc_args)
  except OSError:
    # fork failed
    if sys.version_info[0:3] >= (2, 4, 0):
      print traceback.format_exc()
    else:
      _exc_list = traceback.format_exception(*sys.exc_info())
      print "".join(_exc_list)

    return -1
  return 0

