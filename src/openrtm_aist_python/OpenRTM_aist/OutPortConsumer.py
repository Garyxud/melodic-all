#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file  OutPortConsumer.py
# @brief OutPortConsumer class
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
# @class OutPortConsumer
#
# @brief OutPortConsumer クラス
#
# 出力ポートコンシューマのためのクラス
# 各具象クラスは、以下の関数の実装を提供しなければならない。
# - pull(): データ受信
# - subscribeInterface(): データ受信通知への登録
# - unsubscribeInterface(): データ受信通知の登録解除
#
# @since 0.4.0
#
# @else
# @class OutPortConsumer
# @brief OutPortConsumer class
# @endif
class OutPortConsumer(OpenRTM_aist.DataPortStatus):
  """
  """

  ##
  # @if jp
  # @brief Interface接続用Functor
  # @else
  # @brief Functor to subscribe the interface
  # @endif
  #
  class subscribe:
    # subscribe(const SDOPackage::NVList& prop)
    def __init__(self, prop):
      self._prop = prop
      return

    # void operator()(OutPortConsumer* consumer)
    def __call__(self, consumer):
      consumer.subscribeInterface(self._prop)
      return
    
  ##
  # @if jp
  # @brief Interface接続解除用Functor
  # @else
  # @brief Functor to unsubscribe the interface
  # @endif
  #
  class unsubscribe:
    # unsubscribe(const SDOPackage::NVList& prop)
    def __init__(self, prop):
      self._prop = prop
      return

    # void operator()(OutPortConsumer* consumer)
    def __call__(self, consumer):
      consumer.unsubscribeInterface(self._prop)
      return


outportconsumerfactory = None

class OutPortConsumerFactory(OpenRTM_aist.Factory,OutPortConsumer):
  def __init__(self):
    OpenRTM_aist.Factory.__init__(self)
    pass


  def __del__(self):
    pass


  def instance():
    global outportconsumerfactory

    if outportconsumerfactory is None:
      outportconsumerfactory = OutPortConsumerFactory()

    return outportconsumerfactory

  instance = staticmethod(instance)
