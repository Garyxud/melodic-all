#!/usr/bin/env python 
# -*- coding: euc-jp -*-

##
# @file PublisherBase.py
# @brief Publisher base class
# @date $Date: 2007/09/05$
# @author Noriaki Ando <n-ando@aist.go.jp>
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
# @class PublisherBase
#
# @brief Publisher 基底クラス
# 
# データ送出タイミングを管理して送出を駆動するPublisher* の基底クラス。
# 各種 Publisher はこのクラスを継承して詳細を実装する。
#
# @since 0.4.0
#
# @else
#
# @class PublisherBase
#
# @brief Base class of Publisher.
#
# A base class of Publisher*.
# Variation of Publisher* which implements details of Publisher
# inherits this PublisherBase class.
#
# @endif
class PublisherBase(OpenRTM_aist.DataPortStatus):
  """
  """


  ##
  # @if jp
  # @brief 設定初期化
  #
  # InPortConsumerの各種設定を行う。実装クラスでは、与えられた
  # Propertiesから必要な情報を取得して各種設定を行う。この init() 関
  # 数は、OutPortProvider生成直後および、接続時にそれぞれ呼ばれる可
  # 能性がある。したがって、この関数は複数回呼ばれることを想定して記
  # 述されるべきである。
  # 
  # @param prop 設定情報
  #
  # @else
  #
  # @brief Initializing configuration
  #
  # This operation would be called to configure in initialization.
  # In the concrete class, configuration should be performed
  # getting appropriate information from the given Properties data.
  # This function might be called right after instantiation and
  # connection sequence respectivly.  Therefore, this function
  # should be implemented assuming multiple call.
  #
  # @param prop Configuration information
  #
  # @endif
  ## virtual ReturnCode init(coil::Properties& prop) = 0;
  def init(self, prop):
    pass

  ## virtual ReturnCode setConsumer(InPortConsumer* consumer) = 0;
  def setConsumer(self, consumer):
    pass

  ## virtual ReturnCode setBuffer(BufferBase<cdrMemoryStream>* buffer) = 0;
  def setBuffer(self, buffer):
    pass

  # virtual ReturnCode setListener(ConnectorInfo& info,
  #                                ConnectorListeners* listeners) = 0;
  def setListener(self, info, listeners):
    pass

  # virtual ReturnCode write(const cdrMemoryStream& data,
  #                          unsigned long sec,
  #                          unsigned long usec) = 0;
  def write(self, data, sec, usec):
    pass

  ## virtual bool isActive() = 0;
  def isActive(self):
    pass

  ## virtual ReturnCode activate() = 0;
  def activate(self):
    pass

  ## virtual ReturnCode deactivate() = 0;
  def deactivate(self):
    pass


    
  ##
  # @if jp
  #
  # @brief Publisher を破棄する。
  #
  # 当該 Publisher を破棄する。
  # 当該 Publisher が不要になった場合に PublisherFactory から呼び出される。
  #
  # @else
  #
  # @brief Release the Publisher
  #
  # Release this Publisher.
  # When Publisher becomes unnecessary, this is invoked from
  # PublisherFactory. 
  #
  # @endif
  # virtual void release(){}
  def release(self):
    pass


publisherfactory = None

class PublisherFactory(OpenRTM_aist.Factory,PublisherBase):
  def __init__(self):
    OpenRTM_aist.Factory.__init__(self)
    pass


  def __del__(self):
    pass


  def instance():
    global publisherfactory

    if publisherfactory is None:
      publisherfactory = PublisherFactory()

    return publisherfactory

  instance = staticmethod(instance)
