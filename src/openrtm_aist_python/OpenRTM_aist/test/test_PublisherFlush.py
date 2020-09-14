#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_PublisherFlush.py
#  \brief test for PublisherFlush class
#  \date  $Date: 2007/09/27 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 
import sys
sys.path.insert(1,"../")

import unittest

from PublisherFlush import *
import OpenRTM_aist

class Consumer:
  def put(self,data):
    return data

class TestPublisherNew(unittest.TestCase):

  def setUp(self):
    self._pf = PublisherFlush()
    return

  def test_init(self):
    self.assertEqual(self._pf.init(None),OpenRTM_aist.DataPortStatus.PORT_OK)
    return

  def test_setConsumer(self):
    self.assertEqual(self._pf.setConsumer(None),OpenRTM_aist.DataPortStatus.INVALID_ARGS)
    self.assertEqual(self._pf.init(Consumer()),OpenRTM_aist.DataPortStatus.PORT_OK)
    return

  def test_setBuffer(self):
    self.assertEqual(self._pf.setBuffer(None),OpenRTM_aist.DataPortStatus.PORT_OK)
    return

  def test_write(self):
    prop = OpenRTM_aist.Properties()
    cinfo = OpenRTM_aist.ConnectorInfo("",
                                       "",
                                       [],
                                       prop)
    self.assertEqual(self._pf.write(123,0,0),OpenRTM_aist.DataPortStatus.PRECONDITION_NOT_MET)
    self.assertEqual(self._pf.setConsumer(OpenRTM_aist.InPortCorbaCdrConsumer()),
                     OpenRTM_aist.DataPortStatus.PORT_OK)
    self.assertEqual(self._pf.setListener(cinfo,OpenRTM_aist.ConnectorListeners()),
                     OpenRTM_aist.DataPortStatus.PORT_OK)
    self.assertEqual(self._pf.write(123,0,0),OpenRTM_aist.DataPortStatus.CONNECTION_LOST)
    return

  def test_activate_deactivate_isActive(self):
    self.assertEqual(self._pf.isActive(),False)
    self.assertEqual(self._pf.activate(),OpenRTM_aist.DataPortStatus.PORT_OK)
    self.assertEqual(self._pf.isActive(),True)
    self.assertEqual(self._pf.activate(),OpenRTM_aist.DataPortStatus.PRECONDITION_NOT_MET)
    self.assertEqual(self._pf.deactivate(),OpenRTM_aist.DataPortStatus.PORT_OK)
    self.assertEqual(self._pf.isActive(),False)
    self.assertEqual(self._pf.deactivate(),OpenRTM_aist.DataPortStatus.PRECONDITION_NOT_MET)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
