#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
#
# @file BufferStatus.py
# @brief Buffer status enum definition
# @date $Date: 2007-12-31 03:06:12 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#


class BufferStatus:
  """
  """

  BUFFER_OK            = 0
  BUFFER_ERROR         = 1
  BUFFER_FULL          = 2
  BUFFER_EMPTY         = 3
  NOT_SUPPORTED        = 4
  TIMEOUT              = 5
  PRECONDITION_NOT_MET = 6

  def toString(self, status):
    str = ["BUFFER_OK",
           "BUFFER_ERROR",
           "BUFFER_FULL",
           "BUFFER_EMPTY",
           "NOT_SUPPORTED",
           "TIMEOUT",
           "PRECONDITION_NOT_MET"]
    return str[status]
