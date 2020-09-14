#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# \file DataFlowComponentBase.py
# \brief DataFlowParticipant RT-Component base class
# \date $Date: 2007/09/04$
# \author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import OpenRTM_aist


##
# @if jp
# @class DataFlowComponentBase
# @brief DataFlowComponentBase クラス
#
# データフロー型RTComponentの基底クラス。
# 各種データフロー型RTComponentを実装する場合は、本クラスを継承する形で実装
# する。
#
# @since 0.4.0
#
# @else
# @class DataFlowComponentBase
# @brief DataFlowComponentBase class
# @endif
class DataFlowComponentBase(OpenRTM_aist.RTObject_impl):
  """
  """


  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param manager マネージャオブジェクト
  #
  # @else
  # @brief Constructor
  # @endif
  def __init__(self, manager):
    OpenRTM_aist.RTObject_impl.__init__(self, manager)


  ##
  # @if jp
  # @brief 初期化(サブクラス実装用)
  #
  # データフロー型 RTComponent の初期化を実行する。
  # 実際の初期化処理は、各具象クラス内に記述する。
  #
  # @param self
  #
  # @else
  # @brief Initialization
  # @endif
  def init(self):
    pass


