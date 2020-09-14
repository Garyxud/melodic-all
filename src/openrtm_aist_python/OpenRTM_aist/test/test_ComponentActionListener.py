#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file test_ComponentActionListener.py
# @brief test for ComponentActionListener class
# @date $Date$
# @author Shinji Kurihara
#
# Copyright (C) 2011
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import sys
sys.path.insert(1,"../")

import unittest

from ComponentActionListener import *
import OpenRTM_aist

class MockPreComponentActionListener(PreComponentActionListener):
  def __init__(self):
    PreComponentActionListener.__init__(self)
    return

  def __call__(self,id):
    return id


class MockPostComponentActionListener(PostComponentActionListener):
  def __init__(self):
    PostComponentActionListener.__init__(self)
    return

  def __call__(self,id,ret):
    return id,ret


class MockPortActionListener(PortActionListener):
  def __init__(self):
    PortActionListener.__init__(self)
    return

  def __call__(self, pprof):
    return pprof


class MockExecutionContextActionListener(ExecutionContextActionListener):
  def __init__(self):
    ExecutionContextActionListener.__init__(self)
    return

  def __call__(self, ec_id):
    return ec_id


class TestListener(unittest.TestCase):
  def setUp(self):
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_PreComponentActionListener_toString(self):
    self.assertEqual("PRE_ON_INITIALIZE",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_INITIALIZE))
    
    self.assertEqual("PRE_ON_FINALIZE",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_FINALIZE))

    self.assertEqual("PRE_ON_STARTUP",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_STARTUP))

    self.assertEqual("PRE_ON_SHUTDOWN",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_SHUTDOWN))

    self.assertEqual("PRE_ON_ACTIVATED",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_ACTIVATED))

    self.assertEqual("PRE_ON_DEACTIVATED",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_DEACTIVATED))

    self.assertEqual("PRE_ON_ABORTING",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_ABORTING))

    self.assertEqual("PRE_ON_ERROR",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_ERROR))

    self.assertEqual("PRE_ON_RESET",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_RESET))

    self.assertEqual("PRE_ON_EXECUTE",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_EXECUTE))

    self.assertEqual("PRE_ON_STATE_UPDATE",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_STATE_UPDATE))

    self.assertEqual("PRE_ON_RATE_CHANGED",
                    PreComponentActionListener.toString(
        PreComponentActionListenerType.PRE_ON_RATE_CHANGED))

    return

  def test_PostComponentActionListener_toString(self):
    self.assertEqual("POST_ON_INITIALIZE",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_INITIALIZE))
    
    self.assertEqual("POST_ON_FINALIZE",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_FINALIZE))

    self.assertEqual("POST_ON_STARTUP",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_STARTUP))

    self.assertEqual("POST_ON_SHUTDOWN",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_SHUTDOWN))

    self.assertEqual("POST_ON_ACTIVATED",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_ACTIVATED))

    self.assertEqual("POST_ON_DEACTIVATED",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_DEACTIVATED))

    self.assertEqual("POST_ON_ABORTING",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_ABORTING))

    self.assertEqual("POST_ON_ERROR",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_ERROR))

    self.assertEqual("POST_ON_RESET",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_RESET))

    self.assertEqual("POST_ON_EXECUTE",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_EXECUTE))

    self.assertEqual("POST_ON_STATE_UPDATE",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_STATE_UPDATE))

    self.assertEqual("POST_ON_RATE_CHANGED",
                    PostComponentActionListener.toString(
        PostComponentActionListenerType.POST_ON_RATE_CHANGED))

    return

  def test_PortActionListener_toString(self):
    self.assertEqual("ADD_PORT",
                    PortActionListener.toString(
        PortActionListenerType.ADD_PORT))

    self.assertEqual("REMOVE_PORT",
                    PortActionListener.toString(
        PortActionListenerType.REMOVE_PORT))

    return

  def test_ExecutionContextActionListener_toString(self):
    self.assertEqual("ATTACH_EC",
                    ExecutionContextActionListener.toString(
        ExecutionContextActionListenerType.EC_ATTACHED))

    self.assertEqual("DETACH_EC",
                    ExecutionContextActionListener.toString(
        ExecutionContextActionListenerType.EC_DETACHED))

    return


  def test_PreComponentActionListenerHolder(self):
    preactions = ComponentActionListeners()
    listener = MockPreComponentActionListener()
    preactions.preaction_[0].addListener(listener,True)
    preactions.preaction_[0].notify("test precomp ec_id")
    preactions.preaction_[0].removeListener(listener)
    return

  def test_PostComponentActionListenerHolder(self):
    postactions = ComponentActionListeners()
    listener = MockPostComponentActionListener()
    postactions.postaction_[0].addListener(listener,True)
    postactions.postaction_[0].notify("test postcomp ec_id",True)
    postactions.postaction_[0].removeListener(listener)
    return

  def test_PortActionListenerHolder(self):
    portactions = ComponentActionListeners()
    listener = MockPortActionListener()
    portactions.portaction_[0].addListener(listener,True)
    portactions.portaction_[0].notify("test port pprof")
    portactions.portaction_[0].removeListener(listener)
    return

  def test_ExecutionContextActionListenerHolder(self):
    ecactions = ComponentActionListeners()
    listener = MockExecutionContextActionListener()
    ecactions.ecaction_[0].addListener(listener,True)
    ecactions.ecaction_[0].notify("test ec ec_id")
    ecactions.ecaction_[0].removeListener(listener)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
