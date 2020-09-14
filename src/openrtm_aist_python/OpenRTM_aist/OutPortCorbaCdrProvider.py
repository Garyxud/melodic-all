#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file  OutPortCorbaProvider.py
# @brief OutPortCorbaProvider class
# @date  $Date: 2008-01-14 07:52:40 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
from omniORB import *
from omniORB import any

import OpenRTM_aist
import OpenRTM__POA,OpenRTM

##
# @if jp
# @class OutPortCorbaCdrProvider
# @brief OutPortCorbaCdrProvider クラス
#
# OutPortProvider 
#
# データ転送に CORBA の OpenRTM::OutPortCdr インターフェースを利用し
# た、pull 型データフロー型を実現する OutPort プロバイダクラス。
#
# @since 0.4.0
#
# @else
# @class OutPortCorbaCdrProvider
# @brief OutPortCorbaCdrProvider class
#
# The OutPort provider class which uses the OpenRTM::OutPortCdr
# interface in CORBA for data transfer and realizes a pull-type
# dataflow.
#
# @since 0.4.0
#
# @endif
#
class OutPortCorbaCdrProvider(OpenRTM_aist.OutPortProvider,
                              OpenRTM__POA.OutPortCdr):
  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param buffer 当該プロバイダに割り当てるバッファオブジェクト
  #
  # @else
  # @brief Constructor
  #
  # Constructor
  #
  # @param buffer Buffer object that is assigned to this provider
  #
  # @endif
  #
  def __init__(self):
    OpenRTM_aist.OutPortProvider.__init__(self)
    self.setInterfaceType("corba_cdr")

    # ConnectorProfile setting
    self._objref = self._this()
    
    self._buffer = None

    # set outPort's reference
    orb = OpenRTM_aist.Manager.instance().getORB()

    self._properties.append(OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.outport_ior",
                                                      orb.object_to_string(self._objref)))
    self._properties.append(OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.outport_ref",
                                                      self._objref))

    self._listeners = None
    self._connector = None
    self._profile   = None
    return


  ##
  # @if jp
  # @brief デストラクタ
  #
  # デストラクタ
  #
  # @else
  # @brief Destructor
  #
  # Destructor
  #
  # @endif
  #
  def __del__(self):
    oid = self._default_POA().servant_to_id(self)
    self._default_POA().deactivate_object(oid)
    return


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
  #
  # virtual void init(coil::Properties& prop);
  def init(self, prop):
    pass


  ##
  # @if jp
  # @brief バッファをセットする
  #
  # OutPortProviderがデータを取り出すバッファをセットする。
  # すでにセットされたバッファがある場合、以前のバッファへの
  # ポインタに対して上書きされる。
  # OutPortProviderはバッファの所有権を仮定していないので、
  # バッファの削除はユーザの責任で行わなければならない。
  #
  # @param buffer OutPortProviderがデータを取り出すバッファへのポインタ
  #
  # @else
  # @brief Setting outside buffer's pointer
  #
  # A pointer to a buffer from which OutPortProvider retrieve data.
  # If already buffer is set, previous buffer's pointer will be
  # overwritten by the given pointer to a buffer.  Since
  # OutPortProvider does not assume ownership of the buffer
  # pointer, destructor of the buffer should be done by user.
  # 
  # @param buffer A pointer to a data buffer to be used by OutPortProvider
  #
  # @endif
  #
  # virtual void setBuffer(BufferBase<cdrMemoryStream>* buffer);
  def setBuffer(self, buffer):
    self._buffer = buffer
    return


  ##
  # @if jp
  # @brief リスナを設定する。
  #
  # OutPort はデータ送信処理における各種イベントに対して特定のリスナ
  # オブジェクトをコールするコールバック機構を提供する。詳細は
  # ConnectorListener.h の ConnectorDataListener, ConnectorListener
  # 等を参照のこと。OutPortCorbaCdrProvider では、以下のコールバック
  # が提供される。
  # 
  # - ON_BUFFER_READ
  # - ON_SEND
  # - ON_BUFFER_EMPTY
  # - ON_BUFFER_READ_TIMEOUT
  # - ON_SENDER_EMPTY
  # - ON_SENDER_TIMEOUT
  # - ON_SENDER_ERROR
  #
  # @param info 接続情報
  # @param listeners リスナオブジェクト
  #
  # @else
  # @brief Set the listener. 
  #
  # OutPort provides callback functionality that calls specific
  # listener objects according to the events in the data publishing
  # process. For details, see documentation of
  # ConnectorDataListener class and ConnectorListener class in
  # ConnectorListener.h. In this OutPortCorbaCdrProvider provides
  # the following callbacks.
  # 
  # - ON_BUFFER_READ
  # - ON_SEND
  # - ON_BUFFER_EMPTY
  # - ON_BUFFER_READ_TIMEOUT
  # - ON_SENDER_EMPTY
  # - ON_SENDER_TIMEOUT
  # - ON_SENDER_ERROR
  #
  # @param info Connector information
  # @param listeners Listener objects
  #
  # @endif
  #
  # virtual void setListener(ConnectorInfo& info,
  #                          ConnectorListeners* listeners);
  def setListener(self, info, listeners):
    self._profile = info
    self._listeners = listeners
    return


  ##
  # @if jp
  # @brief Connectorを設定する。
  #
  # OutPort は接続確立時に OutPortConnector オブジェクトを生成し、生
  # 成したオブジェクトのポインタと共にこの関数を呼び出す。所有権は
  # OutPort が保持するので OutPortProvider は OutPortConnector を削
  # 除してはいけない。
  #
  # @param connector OutPortConnector
  #
  # @else
  # @brief set Connector
  #
  # OutPort creates OutPortConnector object when it establishes
  # connection between OutPort and InPort, and it calls this
  # function with a pointer to the connector object. Since the
  # OutPort has the ownership of this connector, OutPortProvider
  # should not delete it.
  #
  # @param connector OutPortConnector
  #
  # @endif
  #
  # virtual void setConnector(OutPortConnector* connector);
  def setConnector(self, connector):
    self._connector = connector
    return


  ##
  # @if jp
  # @brief [CORBA interface] バッファからデータを取得する
  #
  # 設定された内部バッファからデータを取得する。
  #
  # @return 取得データ
  #
  # @else
  # @brief [CORBA interface] Get data from the buffer
  #
  # Get data from the internal buffer.
  #
  # @return Data got from the buffer.
  #
  # @endif
  #
  # virtual ::OpenRTM::PortStatus get(::OpenRTM::CdrData_out data);
  def get(self):
    self._rtcout.RTC_PARANOID("OutPortCorbaCdrProvider.get()")
    if not self._buffer:
      self.onSenderError()
      return (OpenRTM.UNKNOWN_ERROR, None)

    try:
      if self._buffer.empty():
        self._rtcout.RTC_ERROR("buffer is empty.")
        return (OpenRTM.BUFFER_EMPTY, None)

      cdr = [None]
      ret = self._buffer.read(cdr)

      if ret == OpenRTM_aist.BufferStatus.BUFFER_OK:
        if not cdr:
          self._rtcout.RTC_ERROR("buffer is empty.")
          return (OpenRTM.BUFFER_EMPTY, None)
      
    except:
      self._rtcout.RTC_TRACE(OpenRTM_aist.Logger.print_exception())
      return (OpenRTM.UNKNOWN_ERROR, None)

    return self.convertReturn(ret, cdr[0])
    
  ##
  # @if jp
  # @brief ON_BUFFER_READ のリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_BUFFER_READ event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onBufferRead(const cdrMemoryStream& data)
  def onBufferRead(self, data):
    if self._listeners and self._profile:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_READ].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_SEND のリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_SEND event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onSend(const cdrMemoryStream& data)
  def onSend(self, data):
    if self._listeners and self._profile:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_SEND].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_BUFFER_EMPTYのリスナへ通知する。 
  # @else
  # @brief Notify an ON_BUFFER_EMPTY event to listeners
  # @endif
  #
  # inline void onBufferEmpty()
  def onBufferEmpty(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_BUFFER_EMPTY].notify(self._profile)
    return

  ##
  # @if jp
  # @brief ON_BUFFER_READ_TIMEOUT のリスナへ通知する。 
  # @else
  # @brief Notify an ON_BUFFER_READ_TIMEOUT event to listeners
  # @endif
  #
  # inline void onBufferReadTimeout()
  def onBufferReadTimeout(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_BUFFER_READ_TIMEOUT].notify(self._profile)
    return

  ##
  # @if jp
  # @brief ON_SENDER_EMPTYのリスナへ通知する。 
  # @else
  # @brief Notify an ON_SENDER_EMPTY event to listeners
  # @endif
  #
  # inline void onSenderEmpty()
  def onSenderEmpty(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_EMPTY].notify(self._profile)
    return

  ##
  # @if jp
  # @brief ON_SENDER_TIMEOUT のリスナへ通知する。 
  # @else
  # @brief Notify an ON_SENDER_TIMEOUT event to listeners
  # @endif
  #
  # inline void onSenderTimeout()
  def onSenderTimeout(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_TIMEOUT].notify(self._profile)
    return

  ##
  # @if jp
  # @brief ON_SENDER_ERRORのリスナへ通知する。 
  # @else
  # @brief Notify an ON_SENDER_ERROR event to listeners
  # @endif
  #
  # inline void onSenderError()
  def onSenderError(self):
    if self._listeners and self._profile:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_ERROR].notify(self._profile)
    return


  ##
  # @if jp
  # @brief リターンコード変換
  # @else
  # @brief Return codes conversion
  # @endif
  #
  # ::OpenRTM::PortStatus convertReturn(BufferStatus::Enum status,
  #                                     const cdrMemoryStream& data);
  def convertReturn(self, status, data):
    if status == OpenRTM_aist.BufferStatus.BUFFER_OK:
      self.onBufferRead(data)
      self.onSend(data)
      return (OpenRTM.PORT_OK, data)
    
    elif status == OpenRTM_aist.BufferStatus.BUFFER_ERROR:
      self.onSenderError()
      return (OpenRTM.PORT_ERROR, data)
    
    elif status == OpenRTM_aist.BufferStatus.BUFFER_FULL:
      # never come here
      return (OpenRTM.BUFFER_FULL, data)

    elif status == OpenRTM_aist.BufferStatus.BUFFER_EMPTY:
      self.onBufferEmpty()
      self.onSenderEmpty()
      return (OpenRTM.BUFFER_EMPTY, data)

    elif status == OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET:
      self.onSenderError()
      return (OpenRTM.PORT_ERROR, data)
    
    elif status == OpenRTM_aist.BufferStatus.TIMEOUT:
      self.onBufferReadTimeout()
      self.onSenderTimeout()
      return (OpenRTM.BUFFER_TIMEOUT, data)
    
    else:
      return (OpenRTM.UNKNOWN_ERROR, data)
    
    self.onSenderError()
    return (OpenRTM.UNKNOWN_ERROR, data)


def OutPortCorbaCdrProviderInit():
  factory = OpenRTM_aist.OutPortProviderFactory.instance()
  factory.addFactory("corba_cdr",
                     OpenRTM_aist.OutPortCorbaCdrProvider,
                     OpenRTM_aist.Delete)
