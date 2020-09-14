#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file InPortConnector.py
# @brief InPortConnector base class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import OpenRTM_aist
import RTC


##
# @if jp
# @class InPortConnector
# @brief InPortConnector 基底クラス
#
# InPort の Push/Pull 各種 Connector を派生させるための
# 基底クラス。
#
# @since 1.0.0
#
# @else
# @class InPortConnector
# @brief IｎPortConnector base class
#
# The base class to derive subclasses for InPort's Push/Pull Connectors
#
# @since 1.0.0
#
# @endif
#
class InPortConnector(OpenRTM_aist.ConnectorBase):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  # @else
  # @brief Constructor
  # @endif
  #
  # InPortConnector(ConnectorInfo& info,
  #                 CdrBufferBase* buffer);
  def __init__(self, info, buffer):
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("InPortConnector")
    self._profile = info
    self._buffer = buffer
    self._dataType = None
    self._endian = None
    

  ##
  # @if jp
  # @brief デストラクタ
  # @else
  # @brief Destructor
  # @endif
  #
  def __del__(self):
    pass


  ##
  # @if jp
  # @brief ConnectorInfo 取得
  #
  # Connector ConnectorInfo を取得する
  #
  # @else
  # @brief Getting ConnectorInfo
  #
  # This operation returns ConnectorInfo
  #
  # @endif
  #
  # const ConnectorInfo& profile();
  def profile(self):
    self._rtcout.RTC_TRACE("profile()")
    return self._profile


  ##
  # @if jp
  # @brief Connector ID 取得
  #
  # Connector ID を取得する
  #
  # @else
  # @brief Getting Connector ID
  #
  # This operation returns Connector ID
  #
  # @endif
  #
  # const char* id();
  def id(self):
    self._rtcout.RTC_TRACE("id() = %s", self.profile().id)
    return self.profile().id


  ##
  # @if jp
  # @brief Connector 名取得
  #
  # Connector 名を取得する
  #
  # @else
  # @brief Getting Connector name
  #
  # This operation returns Connector name
  #
  # @endif
  #
  # const char* name();
  def name(self):
    self._rtcout.RTC_TRACE("name() = %s", self.profile().name)
    return self.profile().name


  ##
  # @if jp
  # @brief 接続解除関数
  #
  # Connector が保持している接続を解除する
  #
  # @else
  # @brief Disconnect connection
  #
  # This operation disconnect this connection
  #
  # @endif
  #
  # virtual ReturnCode disconnect() = 0;
  def disconnect(self):
    pass

  ##
  # @if jp
  # @brief Buffer を所得する
  #
  # Connector が保持している Buffer を返す
  #
  # @else
  # @brief Getting Buffer
  #
  # This operation returns this connector's buffer
  #
  # @endif
  #
  # virtual CdrBufferBase* getBuffer();
  def getBuffer(self):
    return self._buffer

  ##
  # @if jp
  # @brief read 関数
  #
  # Buffer からデータを InPort へ read する関数
  #
  # @else
  # @brief Destructor
  #
  # The read function to read data from buffer to InPort
  #
  # @endif
  #
  # virtual ReturnCode read(cdrMemoryStream& data) = 0;
  def read(self, data):
    pass

  # void setConnectorInfo(ConnectorInfo profile);
  def setConnectorInfo(self, profile):
    self._profile = profile

    if self._profile.properties.hasKey("serializer"):
      endian = self._profile.properties.getProperty("serializer.cdr.endian")
      if not endian:
        self._rtcout.RTC_ERROR("InPortConnector.setConnectorInfo(): endian is not supported.")
        return RTC.RTC_ERROR
        
      endian = OpenRTM_aist.split(endian, ",") # Maybe endian is ["little","big"]
      endian = OpenRTM_aist.normalize(endian) # Maybe self._endian is "little" or "big"

      if endian == "little":
        self._endian = True
      elif endian == "big":
        self._endian = False
      else:
        self._endian = None
            
    else:
      self._endian = True # little endian

    return RTC.RTC_OK



  # template<class DataType>
  # void setDataTyep(DataType data);
  def setDataType(self, data):
    self._dataType = data
