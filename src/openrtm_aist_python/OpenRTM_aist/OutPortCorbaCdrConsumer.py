#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# @file  OutPortCorbaCdrConsumer.py
# @brief OutPortCorbaCdrConsumer class
# @date  $Date: 2008-01-13 10:28:27 $
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

import sys
from omniORB import any
import OpenRTM_aist
import OpenRTM

##
# @if jp
# @class OutPortCorbaCdrConsumer
#
# @brief OutPortCorbaCdrConsumer クラス
#
# 通信手段に CORBA を利用した出力ポートコンシューマの実装クラス。
#
# @param DataType 本ポートにて扱うデータ型
#
# @since 1.0.0
#
# @else
# @class OutPortCorbaCdrConsumer
#
# @brief OutPortCorbaCdrConsumer class
#
# This is an implementation class of the output Consumer 
# that uses CORBA for means of communication.
#
# @param DataType Data type for this port
#
# @since 1.0.0
#
# @endif
#
class OutPortCorbaCdrConsumer(OpenRTM_aist.OutPortConsumer,OpenRTM_aist.CorbaConsumer):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param buffer 本ポートに割り当てるバッファ
  #
  # @else
  # @brief Constructor
  #
  # Constructor
  #
  # @param buffer Buffer that is attached to this port
  #
  # @endif
  #
  def __init__(self):
    OpenRTM_aist.CorbaConsumer.__init__(self)
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("OutPortCorbaCdrConsumer")
    self._buffer = None
    self._profile = None
    self._listeners = None
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
  def __del__(self, CorbaConsumer=OpenRTM_aist.CorbaConsumer):
    self._rtcout.RTC_PARANOID("~OutPortCorbaCdrConsumer()")
    CorbaConsumer.__del__(self)
    pass


  ##
  # @if jp
  # @brief 設定初期化
  #
  # OutPortConsumerの各種設定を行う。実装クラスでは、与えられた
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
    self._rtcout.RTC_TRACE("init()")
    return


  ##
  # @if jp
  # @brief バッファをセットする
  #
  # OutPortConsumerがデータを取り出すバッファをセットする。
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
  # virtual void setBuffer(CdrBufferBase* buffer);
  def setBuffer(self, buffer):
    self._rtcout.RTC_TRACE("setBuffer()")
    self._buffer = buffer
    return


  # void OutPortCorbaCdrConsumer::setListener(ConnectorInfo& info,
  #                                           ConnectorListeners* listeners)
  def setListener(self, info, listeners):
    self._rtcout.RTC_TRACE("setListener()")
    self._listeners = listeners
    self._profile = info
    return


  ##
  # @if jp
  # @brief データを読み出す
  #
  # 設定されたデータを読み出す。
  #
  # @param data 読み出したデータを受け取るオブジェクト
  #
  # @return データ読み出し処理結果(読み出し成功:true、読み出し失敗:false)
  #
  # @else
  # @brief Read data
  #
  # Read set data
  #
  # @param data Object to receive the read data
  #
  # @return Read result (Successful:true, Failed:false)
  #
  # @endif
  #
  # virtual ReturnCode get(cdrMemoryStream& data);
  def get(self, data):
    self._rtcout.RTC_PARANOID("get()")

    try:
      outportcdr = self.getObject()._narrow(OpenRTM.OutPortCdr)
      ret,cdr_data = outportcdr.get()
      
      if ret == OpenRTM.PORT_OK:
        self._rtcout.RTC_DEBUG("get() successful")
        data[0] = cdr_data
        self.onReceived(data[0])
        self.onBufferWrite(data[0])
        
        if self._buffer.full():
          self._rtcout.RTC_INFO("InPort buffer is full.")
          self.onBufferFull(data[0])
          self.onReceiverFull(data[0])
          
        self._buffer.put(data[0])
        self._buffer.advanceWptr()
        self._buffer.advanceRptr()

        return self.PORT_OK
      return self.convertReturn(ret,data[0])

    except:
      self._rtcout.RTC_WARN("Exception caught from OutPort.get().")
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      return self.CONNECTION_LOST

    self._rtcout.RTC_ERROR("get(): Never comes here.")
    return self.UNKNOWN_ERROR


  ##
  # @if jp
  # @brief データ受信通知への登録
  #
  # 指定されたプロパティに基づいて、データ受信通知の受け取りに登録する。
  #
  # @param properties 登録情報
  #
  # @return 登録処理結果(登録成功:true、登録失敗:false)
  #
  # @else
  # @brief Subscribe the data receive notification
  #
  # Subscribe the data receive notification based on specified property
  # information
  #
  # @param properties Subscription information
  #
  # @return Subscription result (Successful:true, Failed:false)
  #
  # @endif
  #
  # virtual bool subscribeInterface(const SDOPackage::NVList& properties);
  def subscribeInterface(self, properties):
    self._rtcout.RTC_TRACE("subscribeInterface()")
    index = OpenRTM_aist.NVUtil.find_index(properties,"dataport.corba_cdr.outport_ior")
        
    if index < 0:
      self._rtcout.RTC_DEBUG("dataport.corba_cdr.outport_ior not found.")
      return False

    if OpenRTM_aist.NVUtil.isString(properties,"dataport.corba_cdr.outport_ior"):
      self._rtcout.RTC_DEBUG("dataport.corba_cdr.outport_ior found.")
      ior = ""
      try:
        ior = any.from_any(properties[index].value, keep_structs=True)
      except:
        self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
            
      orb = OpenRTM_aist.Manager.instance().getORB()
      obj = orb.string_to_object(ior)
      ret = self.setObject(obj)
      if ret:
        self._rtcout.RTC_DEBUG("CorbaConsumer was set successfully.")
      else:
        self._rtcout.RTC_ERROR("Invalid object reference.")
        
      return ret

    return False


  ##
  # @if jp
  # @brief データ受信通知からの登録解除
  #
  # データ受信通知の受け取りから登録を解除する。
  #
  # @param properties 登録解除情報
  #
  # @else
  # @brief Unsubscribe the data receive notification
  #
  # Unsubscribe the data receive notification.
  #
  # @param properties Unsubscription information
  #
  # @endif
  #
  # virtual void unsubscribeInterface(const SDOPackage::NVList& properties);
  def unsubscribeInterface(self, properties):
    self._rtcout.RTC_TRACE("unsubscribeInterface()")
    index = OpenRTM_aist.NVUtil.find_index(properties,
                                           "dataport.corba_cdr.outport_ref")
    if index < 0:
      self._rtcout.RTC_DEBUG("dataport.corba_cdr.outport_ior not found.")
      return
    
    ior = ""
    try:
      ior = any.from_any(properties[index].value, keep_structs=True)
                
      if ior:
        self._rtcout.RTC_DEBUG("dataport.corba_cdr.outport_ior found.")
        orb = OpenRTM_aist.Manager.instance().getORB()
        obj = orb.string_to_object(ior)
        if self._ptr()._is_equivalent(obj):
          self.releaseObject()
          self._rtcout.RTC_DEBUG("CorbaConsumer's reference was released.")
          return

        self._rtcout.RTC_ERROR("hmm. Inconsistent object reference.")

    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())

    return


  ##
  # @if jp
  # @brief リターンコード変換 (DataPortStatus -> BufferStatus)
  # @else
  # @brief Return codes conversion
  # @endif
  #
  # ReturnCode convertReturn(::OpenRTM::PortStatus status,
  #                          const cdrMemoryStream& data)
  def convertReturn(self, status, data):
    if status == OpenRTM.PORT_OK:
      # never comes here
      return self.PORT_OK

    elif status == OpenRTM.PORT_ERROR:
      self.onSenderError()
      return self.PORT_ERROR

    elif status == OpenRTM.BUFFER_FULL:
      # never comes here
      return self.BUFFER_FULL

    elif status == OpenRTM.BUFFER_EMPTY:
      self.onSenderEmpty()
      return self.BUFFER_EMPTY

    elif status == OpenRTM.BUFFER_TIMEOUT:
      self.onSenderTimeout()
      return self.BUFFER_TIMEOUT

    elif status == OpenRTM.UNKNOWN_ERROR:
      self.onSenderError()
      return self.UNKNOWN_ERROR

    else:
      self.onSenderError()
      return self.UNKNOWN_ERROR

    self.onSenderError()
    return self.UNKNOWN_ERROR

    
  ##
  # @brief Connector data listener functions
  #
  # inline void onBufferWrite(const cdrMemoryStream& data)
  def onBufferWrite(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE].notify(self._profile, data)

    return


  # inline void onBufferFull(const cdrMemoryStream& data)
  def onBufferFull(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_FULL].notify(self._profile, data)

    return


  # inline void onReceived(const cdrMemoryStream& data)
  def onReceived(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED].notify(self._profile, data)

    return


  # inline void onReceiverFull(const cdrMemoryStream& data)
  def onReceiverFull(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_FULL].notify(self._profile, data)

    return


  ##
  # @brief Connector listener functions
  #
  # inline void onSenderEmpty()
  def onSenderEmpty(self):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_EMPTY].notify(self._profile)

    return


  # inline void onSenderTimeout()
  def onSenderTimeout(self):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_TIMEOUT].notify(self._profile)

    return

      
  # inline void onSenderError()
  def onSenderError(self):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_ERROR].notify(self._profile)

    return


def OutPortCorbaCdrConsumerInit():
  factory = OpenRTM_aist.OutPortConsumerFactory.instance()
  factory.addFactory("corba_cdr",
                     OpenRTM_aist.OutPortCorbaCdrConsumer,
                     OpenRTM_aist.Delete)
  return
