#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file Task.py
# @brief Task class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import threading

class Task:
  def __init__(self):
    self._count = 0
    self._thread = None
    return

    
  def __del__(self):
    self._count = 0
    #if self._thread:
    #  if self._thread.isAlive():
    #    self._thread.join()
    self._thread = None
    return


  def open(self, args = None):
    return 0


  def close(self, flags = 0):
    return 0


  def svc(self):
    return 0


  def activate(self):
    if self._count == 0:
      self._thread = threading.Thread(target=self.svc_run)
      self._count += 1
      self._thread.start()
    return


  def wait(self):
    if self._count > 0:
      self._thread.join()
    return


  def suspend(self):
    return 0


  def resume(self):
    return 0


  def reset(self):
    self._count = 0
    return


  def finalize(self):
    self.reset()
    return

  def svc_run(self):
    self.svc()
    self.finalize()
    return
