#/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file Singleton.h
# @brief Singleton template class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import thread

##
# @if jp
# @class Singleton クラステンプレート
#
# このテンプレートは、任意のクラスを Singleton にするテンプレートである。
# 以下のようにして使用する。
#
# class A(Singleton):
#   def __init__(self):
#     pass
class Singleton(object):
  __lockObj = thread.allocate_lock()
  __instance = None

  def __new__(self, *args, **kargs):
    return self.instance(*args, **kargs)


  def __init__(self, *args, **kargs):
    self.instance(*args, **kargs)


  def instance(self, *args, **kargs):
    self.__lockObj.acquire()
    try:
      if self.__instance is None:
        self.__instance = object.__new__(self, *args, **kargs)

    finally:
      self.__lockObj.release()

    return self.__instance

  instance = classmethod(instance)

