#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

##
# @file Composite.cpp
# @brief RT component server daemon
# @date $Date: 2009-01-15 09:06:19 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#


import sys

import OpenRTM_aist

def main():
  manager = OpenRTM_aist.Manager.init(sys.argv)

  manager.activateManager()
  
  manager.runManager()

  return 0


if __name__ == "__main__":
  main()
