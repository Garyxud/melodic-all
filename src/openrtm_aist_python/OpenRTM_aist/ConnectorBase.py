#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ConnectorBase.py
# @brief Connector base class
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

import OpenRTM_aist


##
# @if jp
# @class ConnectorInfo クラス
# @brief ConnectorInfo クラス
#
# @class ConnectorInfo class
# @brief ConnectorInfo class
#
# @endif
#
class ConnectorInfo:
  """
  """
  ##
  # @if jp
  #
  # @brief コンストラクタ
  # 
  # コンストラクタ
  #
  # @param name_ 接続名前
  # @param id_ 接続ID
  # @param ports_ 接続ポートIOR
  # @param properties_ プロパティ
  # 
  # @else
  #
  # @brief Constructor
  # 
  # Constructor
  #
  # @param name_ connection name
  # @param id_ connection ID
  # @param ports_ connection Ports
  # @param properties_ connection properties
  #
  # @endif
  # ConnectorInfo(const char* name_, const char* id_,
  #               coil::vstring ports_, coil::Properties properties_)
  def __init__(self, name_, id_, ports_, properties_):
    self.name       = name_        # str
    self.id         = id_          # str
    self.ports      = ports_       # [str,...]
    self.properties = properties_  # OpenRTM_aist.Properties

#!
# @if jp
# @class ConnectorBase
# @brief Connector 基底クラス
#
# InPort/OutPort, Push/Pull 各種 Connector を派生させるための
# 基底クラス。
#
# @since 1.0.0
#
# @else
# @class ConnectorBase
# @brief Connector Base class
#
# The base class to derive subclasses for InPort/OutPort,
# Push/Pull Connectors
#
# @since 1.0.0
#
# @endif
class ConnectorBase(OpenRTM_aist.DataPortStatus):
  """
  """

  ##
  # @if jp
  # @class Profile
  # @brief Connector profile ローカル構造体
  #
  # ConnectorBase およびその派生クラスで扱う ConnectorProfile 構造体。
  #
  # @since 1.0.0
  #
  # @else
  # @class Profile
  # @brief local representation of Connector profile
  #
  # ConnectorProfile struct for ConnectorBase and its subclasses.
  #
  # @since 1.0.0
  #
  # @endif

  ##
  # @if jp
  # @brief デストラクタ
  # @else
  # @brief Destructor
  # @endif
  def __del__(self):
    pass


  ##
  # @if jp
  # @brief Profile 取得
  #
  # Connector Profile を取得する
  #
  # @else
  # @brief Getting Profile
  #
  # This operation returns Connector Profile
  #
  # @endif
  # virtual const ConnectorInfo& profile() = 0;
  def profile(self):
    pass

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
  # virtual const char* id() = 0;
  def id(self):
    pass


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
  # virtual const char* name() = 0;
  def name(self):
    pass


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
  # virtual ReturnCode disconnect() = 0;
  def disconnect(self):
    pass


  ##
  # @if jp
  # @brief Buffer を取得する
  #
  # Connector が保持している Buffer を返す
  #
  # @else
  # @brief Getting Buffer
  #
  # This operation returns this connector's buffer
  #
  # @endif
  # virtual CdrBufferBase* getBuffer() = 0;
  def getBuffer(self):
    pass


  ##
  # @if jp
  # @brief アクティブ化
  #
  # このコネクタをアクティブ化する
  #
  # @else
  #
  # @brief Connector activation
  #
  # This operation activates this connector
  #
  # @endif
  # virtual void activate() = 0;
  def activate(self):
    pass

  
  ##
  # @if jp
  # @brief 非アクティブ化
  #
  # このコネクタを非アクティブ化する
  #
  # @else
  #
  # @brief Connector deactivation
  #
  # This operation deactivates this connector
  #
  # @endif
  # virtual void deactivate() = 0;
  def deactivate(self):
    pass
