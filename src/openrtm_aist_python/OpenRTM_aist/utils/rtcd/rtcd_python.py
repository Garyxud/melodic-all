#!/usr/bin/env python
# -*- Python -*-

##
# @file rtcd.py
# @brief RT component server daemon
# @date $Date: 2005-05-12 09:06:19 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: $

import sys,os

import OpenRTM_aist

def main():
  conf_file=os.path.join(os.path.dirname(__file__),"rtcd.conf")
  sys.argv.append('-f')
  sys.argv.append(conf_file)
  manager = OpenRTM_aist.Manager.init(sys.argv)

  manager.activateManager()

  manager.runManager()

  return

if __name__ == "__main__":
  main()
