#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file Listener.py
# @brief Listener class
# @date $Date: 2007/08/23$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2007-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.



##
# @if jp
# @class ListenerBase
# @brief ListenerBase クラス
#
# タイマーに登録するリスナー用抽象インターフェースクラス。
#
# @since 0.4.0
#
# @else
# @class ListenerBase
# @brief ListenerBase class
# @endif
class ListenerBase:
  """
  """

  ##
  # @if jp
  # @brief コールバック処理(サブクラス実装用)
  #
  # コールバック処理用関数<BR>
  # ※サブクラスでの実装参照用
  #
  # @param self
  #
  # @else
  #
  # @endif
  def invoke(self):
    pass



##
# @if jp
# @class ListenerObject
# @brief ListenerObject クラス
#
# タイマーに登録するリスナー用基底クラス。
#
# @since 0.4.0
#
# @else
# @class ListenerObject
# @brief ListenerObject class
# @endif
class ListenerObject(ListenerBase):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param obj リスナーオブジェクト
  # @param cbf コールバック用関数
  #
  # @else
  #
  # @endif
  def __init__(self,obj,cbf):
    self.obj = obj
    self.cbf = cbf


  ##
  # @if jp
  # @brief コールバック用処理
  #
  # コールバック処理用関数
  #
  # @param self
  #
  # @else
  #
  # @endif
  def invoke(self):
    self.cbf(self.obj)



##
# @if jp
# @class ListenerFunc
# @brief ListenerFunc クラス
#
# コールバック用オブジェクト。
#
# @since 0.4.0
#
# @else
# @class ListenerFunc
# @brief ListenerFunc class
# @endif
class ListenerFunc(ListenerBase):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param cbf コールバック用関数
  #
  # @else
  #
  # @endif
  def __init__(self,cbf):
    self.cbf = cbf


  ##
  # @if jp
  # @brief コールバック処理
  #
  # コールバック処理用関数
  #
  # @param self
  #
  # @else
  #
  # @endif
  def invoke(self):
    self.cbf()
