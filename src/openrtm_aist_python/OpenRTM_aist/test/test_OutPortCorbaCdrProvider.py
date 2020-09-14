#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_OutPortCorbaCdrProvider.py
#  \brief test for OutPortCorbaCdrProvider class
#  \date  $Date: 2007/09/26 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 
from omniORB import any
from omniORB import CORBA

import OpenRTM_aist
import RTC, RTC__POA 
import SDOPackage, SDOPackage__POA

import sys
sys.path.insert(1,"../")

import unittest

from OutPortCorbaCdrProvider import *

class DummyBuffer:
  def __init__(self):
    self._cdr = None
    self._empty = True

  def empty(self):
    return self._empty

  def write(self,d):
    self._cdr = d
    self._empty = False
    return 0

  def read(self,cdr):
    cdr[0] = self._cdr
    self._empty = True
    return 0

class TestOutPortCorbaCdrProvider(unittest.TestCase):

  def setUp(self):
    OpenRTM_aist.Manager.instance()
    OpenRTM_aist.OutPortCorbaCdrProviderInit()
    self._opp = OpenRTM_aist.OutPortCorbaCdrProvider()
    return
    
  def test_setBuffer(self):
    self._opp.setBuffer(DummyBuffer())
    return

  def test_get(self):
    ret,data=self._opp.get()
    self.assertEqual(ret,OpenRTM.UNKNOWN_ERROR)

    prop = OpenRTM_aist.Properties()
    cinfo = OpenRTM_aist.ConnectorInfo("",
                                       "",
                                       [],
                                       prop)
    self._opp.setListener(cinfo,OpenRTM_aist.ConnectorListeners())
    buff = DummyBuffer()
    self._opp.setBuffer(buff)
    ret,data=self._opp.get()
    self.assertEqual(ret,OpenRTM.BUFFER_EMPTY)

    buff.write(123)
    ret,data=self._opp.get()
    self.assertEqual(data,123)
    return


############### test #################
if __name__ == '__main__':
        unittest.main()
