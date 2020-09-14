#!/usr/bin/env python
# -*- Python -*-


## \file MySdoServiceConsumer.py
## \brief test for SdoServiceConsumer class
## \date $Date: $
## \author Shinji Kurihara
#
# Copyright (C) 2011
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")
sys.path.insert(1,"../RTM_IDL")
import OpenRTM_aist
import OpenRTM


class MySdoServiceConsumer(OpenRTM_aist.SdoServiceConsumerBase):
  def __init__(self):
    self._profile = None
    self._rtobj = None
    return

  def __del__(self):
    return

  def init(self, rtobj, profile):
    self._rtobj = rtobj
    self._profile = profile
    return True

  def reinit(self, profile):
    return

  def getProfile(self):
    return self._profile

  def finalize(self):
    return


def MySdoServiceConsumerInit(manager):
  factory = OpenRTM_aist.SdoServiceConsumerFactory.instance()
  factory.addFactory(OpenRTM_aist.toTypename(OpenRTM.ComponentObserver),
                     MySdoServiceConsumer,
                     OpenRTM_aist.Delete)
  return
