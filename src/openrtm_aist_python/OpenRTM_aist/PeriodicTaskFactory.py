#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file PeriodicTaskFactory.py
# @brief PeiodicTaskFactory class
# @date $Date$
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

import OpenRTM_aist

periodictaskfactory = None

class PeriodicTaskFactory(OpenRTM_aist.Factory,OpenRTM_aist.PeriodicTask):
  def __init__(self):
    OpenRTM_aist.Factory.__init__(self)
    pass


  def __del__(self):
    pass


  def instance():
    global periodictaskfactory

    if periodictaskfactory is None:
      periodictaskfactory = PeriodicTaskFactory()

    return periodictaskfactory

  instance = staticmethod(instance)
