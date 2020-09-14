#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# @file  InPortConsumer.py
# @brief InPortConsumer class
# @date  $Date: 2007/09/04$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import OpenRTM_aist

##
# @if jp
#
# @class InPortConsumer
#
# @brief InPortConsumer 基底クラス
#
# 入力ポートコンシューマのための抽象インターフェースクラス
# 各具象クラスは、以下の関数の実装を提供しなければならない。
# - push(): データ送信
# - clone(): ポートのコピー
# - subscribeInterface(): データ送出通知への登録
# - unsubscribeInterface(): データ送出通知の登録解除
#
# @since 0.4.0
#
# @else
# @class InPortConsumer
# @brief InPortConsumer class
# @endif
class InPortConsumer(OpenRTM_aist.DataPortStatus):
  """
  """



  ##
  # @if jp
  # @brief インターフェースプロファイルを公開するたのファンクタ
  # @else
  # @brief Functor to publish interface profile
  # @endif
  #
  class publishInterfaceProfileFunc:
    def __init__(self, prop):
      self._prop = prop

    def __call__(self, consumer):
      consumer.publishInterfaceProfile(self._prop)


  ##
  # @if jp
  # @brief インターフェースプロファイルを公開するたのファンクタ
  # @else
  # @brief Functor to publish interface profile
  # @endif
  #
  class subscribeInterfaceFunc:
    def __init__(self, prop):
      self._prop = prop

    def __call__(self, consumer):
      return consumer.subscribeInterface(self._prop)


inportconsumerfactory = None

class InPortConsumerFactory(OpenRTM_aist.Factory,InPortConsumer):

  def __init__(self):
    OpenRTM_aist.Factory.__init__(self)
    pass


  def __del__(self):
    pass


  def instance():
    global inportconsumerfactory

    if inportconsumerfactory is None:
      inportconsumerfactory = InPortConsumerFactory()

    return inportconsumerfactory

  instance = staticmethod(instance)

