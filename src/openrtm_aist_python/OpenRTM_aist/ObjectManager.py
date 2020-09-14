#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ObjectManager.py
# @brief Object management class
# @date $Date: $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import sys
import string
import threading

import OpenRTM_aist



##
# @if jp
#
# @brief オブジェクト管理用クラス
#
# 各種オブジェクトを管理するためのクラス。
#
# @since 0.4.0
#
# @else
#
# @endif
class ObjectManager:
  """
  """

  ##
  # @if jp
  #
  # @brief コンストラクタ
  # 
  # コンストラクタ
  # 
  # @param self
  # @param predicate オブジェクト検索用ファンクタ
  # 
  # @else
  #
  # @endif
  def __init__(self, predicate):
    self._objects = self.Objects()
    self._predicate = predicate



  ##
  # @if jp
  # @class Objects
  # @brief オブジェクト管理用内部クラス
  # @endif
  class Objects:
    def __init__(self):
      self._mutex = threading.RLock()
      self._obj = []


  ##
  # @if jp
  #
  # @brief 指定したオブジェクトを登録する
  # 
  # 指定したオブジェクトを登録する。
  # 同一オブジェクトが登録済みの場合は、何も行わない。
  #
  # @param self
  # @param obj 登録対象オブジェクト
  #
  # @return 登録処理結果(オブジェクトを登録した場合にtrue)
  # 
  # @else
  #
  # @endif
  def registerObject(self, obj):
    guard = OpenRTM_aist.ScopedLock(self._objects._mutex)
    predi = self._predicate(factory=obj)

    for _obj in self._objects._obj:
      if predi(_obj):
        return False

    self._objects._obj.append(obj)
    return True


  ##
  # @if jp
  #
  # @brief 指定したオブジェクトを登録解除する
  # 
  # 指定したオブジェクトの登録を解除し、取得する。
  # 指定したオブジェクトが登録されていない場合にはNULLを返す。
  #
  # @param self
  # @param id 登録解除対象オブジェクトのID
  #
  # @return 登録解除されたオブジェクト
  # 
  # @else
  #
  # @endif
  def unregisterObject(self, id):
    guard = OpenRTM_aist.ScopedLock(self._objects._mutex)
    predi = self._predicate(name=id)
    i = 0
    for _obj in self._objects._obj:
      if predi(_obj):
        ret = _obj
        del self._objects._obj[i]
        return ret
      i+=1
      
    return None


  ##
  # @if jp
  #
  # @brief オブジェクトを検索する
  # 
  # 登録されているオブジェクトの中から指定した条件に合致するオブジェクトを検索
  # して取得する。
  # 指定した条件に合致するオブジェクトが登録されていない場合にはNULLを返す。
  #
  # @param self
  # @param id 検索対象オブジェクトのID
  #
  # @return オブジェクトの検索結果
  # 
  # @else
  #
  # @endif
  def find(self, id):
    guard = OpenRTM_aist.ScopedLock(self._objects._mutex)
    if isinstance(id,str):
      predi = self._predicate(name=id)
    else:
      predi = self._predicate(prop=id)

    for _obj in self._objects._obj:
      if predi(_obj):
        return _obj
      
    return None


  ##
  # @if jp
  #
  # @brief 登録されているオブジェクトのリストを取得する
  # 
  # 登録されているオブジェクトのリストを取得する。
  #
  # @param self
  #
  # @return 登録されているオブジェクト・リスト
  # 
  # @else
  #
  # @endif
  def getObjects(self):
    guard = OpenRTM_aist.ScopedLock(self._objects._mutex)
    return self._objects._obj


  ##
  # @if jp
  # @brief オブジェクトを検索する
  #
  # 指定された条件に合致するオブジェクトを検索する。
  #
  # @param self
  # @param p オブジェクト検索用ファンクタ
  #
  # @else
  #
  # @endif
  def for_each(self,p):
    guard = OpenRTM_aist.ScopedLock(self._objects._mutex)
    predi = p()

    for _obj in self._objects._obj:
      predi(_obj)

    return predi

