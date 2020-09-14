#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file  CdrRingBuffer.py
# @brief RingBuffer for CDR
# @date  $Date: 2007-12-31 03:08:03 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import OpenRTM_aist
from OpenRTM_aist import *

class CdrRingBuffer(OpenRTM_aist.RingBuffer):
  """
  """
  def __init__(self):
    OpenRTM_aist.RingBuffer.__init__(self)
    pass


def CdrRingBufferInit():
  OpenRTM_aist.CdrBufferFactory.instance().addFactory("ring_buffer",
                                                      OpenRTM_aist.CdrRingBuffer,
                                                      OpenRTM_aist.Delete)
