#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file NumberingPolicy.py
# @brief Object numbering policy class
# @date $Date: 2007/08/23$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import string
import OpenRTM_aist

##
# @if jp
#
# @class NumberingPolicy
# @brief オブジェクト生成時ネーミング・ポリシー(命名規則)管理用抽象クラス
#
# オブジェクトを生成する際のネーミング・ポリシー(命名規則)を管理するための
# 抽象クラス。
# 具象クラスは、以下の関数の実装を提供しなければならない。
# - onCreate() : オブジェクト生成時の名称作成
# - onDelete() : オブジェクト削除時の名称解放
#
# @since 0.4.0
#
# @else
#
# @endif
class NumberingPolicy:
  """
  """



  ##
  # @if jp
  # @brief オブジェクト未発見例外処理用内部クラス(未実装)
  # @else
  #
  # @endif
  class ObjectNotFound:
    pass


  ##
  # @if jp
  #
  # @brief オブジェクト生成時の名称作成(サブクラス実装用)
  #
  # オブジェクト生成時の名称を生成するための関数<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self
  # @param obj 名称生成対象オブジェクト
  #
  # @return 生成したオブジェクト名称
  #
  # @else
  #
  # @endif
  def onCreate(self, obj):
    pass


  ##
  # @if jp
  #
  # @brief オブジェクト削除時の名称解放(サブクラス実装用)
  #
  # オブジェクト削除時に名称を解放するための関数<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self
  # @param obj 名称解放対象オブジェクト
  #
  # @else
  #
  # @endif
  def onDelete(self, obj):
    pass



##
# @if jp
#
# @class DefaultNumberingPolicy
# @brief オブジェクト生成時ネーミング・ポリシー(命名規則)管理用クラス
#
# オブジェクトを生成する際のネーミング・ポリシー(命名規則)を管理するための
# クラス。
#
# @since 0.4.0
#
# @else
#
# @endif
class DefaultNumberingPolicy(NumberingPolicy):
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
  # 
  # @else
  #
  # @brief virtual destractor
  #
  # @endif
  def __init__(self):
    self._num = 0
    self._objects = []


  ##
  # @if jp
  #
  # @brief オブジェクト生成時の名称作成
  #
  # オブジェクト生成時の名称を生成する。
  # 生成済みインスタンスの数に応じた名称を生成する。
  # 
  # @param self
  # @param obj 名称生成対象オブジェクト
  #
  # @return 生成したオブジェクト名称
  #
  # @else
  #
  # @endif
  def onCreate(self, obj):
    self._num += 1

    pos = 0
    try:
      pos = self.find(None)
      self._objects[pos] = obj
      return OpenRTM_aist.otos(pos)
    except NumberingPolicy.ObjectNotFound:
      self._objects.append(obj)
      return OpenRTM_aist.otos(int(len(self._objects) - 1))


  ##
  # @if jp
  #
  # @brief オブジェクト削除時の名称解放
  #
  # オブジェクト削除時に名称を解放する。
  # オブジェクト削除時に生成済みインスタンス数を減算する。
  # 
  # @param self
  # @param obj 名称解放対象オブジェクト
  #
  # @else
  #
  # @endif
  def onDelete(self, obj):
    pos = 0
    try:
      pos = self.find(obj)
    except:
      return

    if (pos < len(self._objects)):
      self._objects[pos] = None
    self._num -= 1


  ##
  # @if jp
  #
  # @brief オブジェクトの検索
  #
  # オブジェクトリストから指定されたオブジェクトを検索し、
  # 該当するオブジェクトが格納されている場合にはインデックスを返す。
  # 
  # @param self
  # @param obj 検索対象オブジェクト
  #
  # @return オブジェクト格納インデックス
  #
  # @else
  #
  # @endif
  def find(self, obj):
    i = 0
    for obj_ in self._objects:
      if obj_ == obj:
        return i
      i += 1
    raise NumberingPolicy.ObjectNotFound()
       

