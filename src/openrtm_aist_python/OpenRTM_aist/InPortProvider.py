#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file  InPortProvider.py
# @brief InPortProvider class
# @date  $Date: 2007/09/20 $
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
import SDOPackage, SDOPackage__POA


##
# @if jp
# @class InPortProvider
# @brief InPortProvider クラス
#
# InPortの情報を保持するためのクラス。
#
# @since 0.4.0
#
# @else
# @class InPortProvider
# @brief InPortProvider class
# @endif
class InPortProvider(OpenRTM_aist.DataPortStatus):
  """
  """



  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  #
  # @else
  # @brief Constructor
  # @endif
  def __init__(self):
    self._properties = []
    self._interfaceType = ""
    self._dataflowType = ""
    self._subscriptionType = ""
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("InPortProvider")
    self._connector = None


  ##
  # @if jp
  # @brief InterfaceProfile情報を公開する
  #
  # InterfaceProfile情報を公開する。
  #
  # @param self
  # @param prop InterfaceProfile情報を受け取るプロパティ
  #
  # @else
  #
  # @endif
  def publishInterfaceProfile(self, prop):
    self._rtcout.RTC_TRACE("publishInterfaceProfile()")
    OpenRTM_aist.NVUtil.appendStringValue(prop, "dataport.interface_type",
                                          self._interfaceType)
    OpenRTM_aist.NVUtil.append(prop, self._properties)

  ##
  # @if jp
  # @brief Interface情報を公開する
  #
  # Interface情報を公開する。
  #
  # @param self
  # @param prop Interface情報を受け取るプロパティ
  #
  # @else
  #
  # @endif
  def publishInterface(self, prop):
    self._rtcout.RTC_TRACE("publishInterface()")
    if not OpenRTM_aist.NVUtil.isStringValue(prop,
                                             "dataport.interface_type",
                                             self._interfaceType):
      return False

    OpenRTM_aist.NVUtil.append(prop, self._properties)
    return True


  ##
  # @if jp
  # @brief インタフェースタイプを設定する
  #
  # インタフェースタイプを設定する。
  #
  # @param self
  # @param interface_type 設定対象インタフェースタイプ
  #
  # @else
  #
  # @endif
  def setInterfaceType(self, interface_type):
    self._rtcout.RTC_TRACE("setInterfaceType(%s)", interface_type)
    self._interfaceType = interface_type


  ##
  # @if jp
  # @brief データフロータイプを設定する
  #
  # データフロータイプを設定する。
  #
  # @param self
  # @param dataflow_type 設定対象データフロータイプ
  #
  # @else
  #
  # @endif
  def setDataFlowType(self, dataflow_type):
    self._rtcout.RTC_TRACE("setDataFlowType(%s)", dataflow_type)
    self._dataflowType = dataflow_type


  ##
  # @if jp
  # @brief サブスクリプションタイプを設定する
  #
  # サブスクリプションタイプを設定する。
  #
  # @param self
  # @param subs_type 設定対象サブスクリプションタイプ
  #
  # @else
  #
  # @endif
  def setSubscriptionType(self, subs_type):
    self._rtcout.RTC_TRACE("setSubscriptionType(%s)", subs_type)
    self._subscriptionType = subs_type



  def setConnector(self, connector):
    self._connector = connector


inportproviderfactory = None


class InPortProviderFactory(OpenRTM_aist.Factory,InPortProvider):
  def __init__(self):
    OpenRTM_aist.Factory.__init__(self) # Call GlobalFactory.Factory()
    InPortProvider.__init__(self)
    return


  def instance():
    global inportproviderfactory

    if inportproviderfactory is None:
      inportproviderfactory = InPortProviderFactory()

    return inportproviderfactory

  instance = staticmethod(instance)


  def __del__(self):
    pass
