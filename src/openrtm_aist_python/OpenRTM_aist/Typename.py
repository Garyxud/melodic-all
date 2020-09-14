#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file Typename.py
# @brief Typename function
# @date $Date: 2007-12-31 03:08:03 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

from omniORB import *
from omniORB import any

import OpenRTM_aist

## const char* toTypename(value)
def toTypename(value):
  return str(value._NP_RepositoryId)
