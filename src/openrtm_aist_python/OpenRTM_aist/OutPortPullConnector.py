#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# @file OutPortPullConnector.py
# @brief OutPortPull type connector class
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

from omniORB import *
from omniORB import any

import OpenRTM_aist


##
# @if jp
# @class OutPortPullConnector
# @brief OutPortPullConnector クラス
#
# OutPort の pull 型データフローのための Connector クラス。このオブ
# ジェクトは、接続時に dataflow_type に pull が指定された場合、
# OutPort によって生成・所有され、InPortPullConnector と対になって、
# データポートの pull 型のデータフローを実現する。一つの接続に対して、
# 一つのデータストリームを提供する唯一の Connector が対応する。
# Connector は 接続時に生成される UUID 形式の ID により区別される。
#
# OutPortPullConnector は以下の三つのオブジェクトを所有し管理する。
#
# - InPortConsumer
# - Buffer
#
# OutPort に書き込まれたデータは OutPortPullConnector::write() に渡
# され Buffer に書き込まれる。InPortPullConnector が
# OutPortPullConnector からデータを読み出すことで InPort にデータが
# 転送される。
#
# @since 1.0.0
#
# @else
# @class OutPortPullConnector
# @brief OutPortPullConnector class
#
# Connector class of OutPort for pull type dataflow. When "pull" is
# specified as dataflow_type at the time of establishing
# connection, this object is generated and owned by the OutPort.
# This connector and InPortPullConnector make a pair and realize
# pull type dataflow of data ports. One connector corresponds to
# one connection which provides a data stream. Connector is
# distinguished by ID of the UUID that is generated at establishing
# connection.
#
# OutPortPullConnector owns and manages the following objects.
#
# - InPortConsumer
# - Buffer
#
# Data written into the OutPort is passed to
# OutPortPullConnector::write(), and it is written into the buffer.
# By reading data from OutPortPullConnector to InPortPullConnector,
# data transfer is realized.
#
# @since 1.0.0
#
# @endif
#
class OutPortPullConnector(OpenRTM_aist.OutPortConnector):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # OutPortPullConnector のコンストラクタはオブジェクト生成時に下記
  # を引数にとる。ConnectorInfo は接続情報を含み、この情報に従いバッ
  # ファ等を生成する。OutPort インターフェースのプロバイダオブジェク
  # トへのポインタを取り、所有権を持つので、OutPortPullConnector は
  # OutPortProvider の解体責任を持つ。各種イベントに対するコールバッ
  # ク機構を提供する ConnectorListeners を持ち、適切なタイミングでコー
  # ルバックを呼び出す。データバッファがもし OutPortBase から提供さ
  # れる場合はそのポインタを取る。
  #
  # @param info ConnectorInfo
  # @param provider OutPortProvider
  # @param listeners ConnectorListeners 型のリスナオブジェクトリスト
  # @param buffer CdrBufferBase 型のバッファ
  #
  # @else
  # @brief Constructor
  #
  # OutPortPullConnector's constructor is given the following
  # arguments.  According to ConnectorInfo which includes
  # connection information, a buffer is created.  It is also given
  # a pointer to the provider object for the OutPort interface.
  # The owner-ship of the pointer is owned by this
  # OutPortPullConnector, it has responsibility to destruct the
  # OutPortProvider.  OutPortPullConnector also has
  # ConnectorListeners to provide event callback mechanisms, and
  # they would be called at the proper timing.  If data buffer is
  # given by OutPortBase, the pointer to the buffer is also given
  # as arguments.
  #
  # @param info ConnectorInfo
  # @param provider OutPortProvider
  # @param listeners ConnectorListeners type lsitener object list
  # @param buffer CdrBufferBase type buffer
  #
  # @endif
  #
  # OutPortPullConnector(ConnectorInfo info,
  #                      OutPortProvider* provider,
  #                      ConnectorListeners& listeners,
  #                      CdrBufferBase* buffer = 0);
  def __init__(self, info, provider, listeners, buffer = 0):
    OpenRTM_aist.OutPortConnector.__init__(self, info)
    self._provider = provider
    self._listeners = listeners
    self._buffer = buffer

    if not self._buffer:
      self._buffer = self.createBuffer(info)

    if not self._provider or not self._buffer:
      self._rtcout.RTC_ERROR("Exeption: in OutPortPullConnector.__init__().")
      raise

    self._buffer.init(info.properties.getNode("buffer"))
    self._provider.setBuffer(self._buffer)
    self._provider.setConnector(self)
    self._provider.setListener(info, self._listeners)
    self.onConnect()
    return


  ##
  # @if jp
  # @brief デストラクタ
  #
  # disconnect() が呼ばれ、provider, buffer が解体・削除される。
  #
  # @else
  #
  # @brief Destructor
  #
  # This operation calls disconnect(), which destructs and deletes
  # the consumer, the publisher and the buffer.
  #
  # @endif
  #
  def __del__(self):
    return


  ##
  # @if jp
  # @brief データの書き込み
  #
  # Publisherに対してデータを書き込み、これにより対応するInPortへ
  # データが転送される。
  #
  # @else
  #
  # @brief Writing data
  #
  # This operation writes data into publisher and then the data
  # will be transferred to correspondent InPort.
  #
  # @endif
  #
  # virtual ReturnCode write(const cdrMemoryStream& data);
  def write(self, data):
    # data -> (conversion) -> CDR stream
    cdr_data = None
    if self._endian is not None:
      cdr_data = cdrMarshal(any.to_any(data).typecode(), data, self._endian)
    else:
      self._rtcout.RTC_ERROR("write(): endian %s is not support.",self._endian)
      return self.UNKNOWN_ERROR
    
    self._buffer.write(cdr_data)
    return self.PORT_OK


  ##
  # @if jp
  # @brief 接続解除
  #
  # consumer, publisher, buffer が解体・削除される。
  #
  # @else
  #
  # @brief disconnect
  #
  # This operation destruct and delete the consumer, the publisher
  # and the buffer.
  #
  # @endif
  #
  # virtual ReturnCode disconnect();
  def disconnect(self):
    self._rtcout.RTC_TRACE("disconnect()")
    self.onDisconnect()
    # delete provider
    if self._provider:
      OpenRTM_aist.OutPortProviderFactory.instance().deleteObject(self._provider)
    self._provider = 0

    # delete buffer
    if self._buffer:
      OpenRTM_aist.CdrBufferFactory.instance().deleteObject(self._buffer)
    self._buffer = 0

    return self.PORT_OK


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
  #
  # virtual CdrBufferBase* getBuffer();
  def getBuffer(self):
    return self._buffer


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
  #
  # virtual void activate(){}; // do nothing
  def activate(self):  # do nothing
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
  #
  # virtual void deactivate(){}; // do nothing
  def deactivate(self): # do nothing
    pass

    
  ##
  # @if jp
  # @brief Bufferの生成
  # @else
  # @brief create buffer
  # @endif
  #
  # CdrBufferBase* createBuffer(ConnectorInfo& info);
  def createBuffer(self, info):
    buf_type = info.properties.getProperty("buffer_type","ring_buffer")
    return OpenRTM_aist.CdrBufferFactory.instance().createObject(buf_type)


  ##
  # @if jp
  # @brief 接続確立時にコールバックを呼ぶ
  # @else
  # @brief Invoke callback when connection is established
  # @endif
  # void onConnect()
  def onConnect(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_CONNECT].notify(self._profile)
    return


  ##
  # @if jp
  # @brief 接続切断時にコールバックを呼ぶ
  # @else
  # @brief Invoke callback when connection is destroied
  # @endif
  # void onDisconnect()
  def onDisconnect(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_DISCONNECT].notify(self._profile)
    return
