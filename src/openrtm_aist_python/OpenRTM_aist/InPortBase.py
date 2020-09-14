#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file InPortBase.py
# @brief RTC::Port implementation for InPort
# @date $Date: 2008-01-13 15:06:40 $
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

import copy
import threading
import OpenRTM_aist
import RTC

##
# @if jp
# @namespace RTC
#
# @brief RTコンポーネント
#
# @else
#
# @namespace RTC
#
# @brief RT-Component
#
# @endif
#

##
# @if jp
# @class InPortBase
# @brief InPort 用 Port
#
# データ入力ポートの実装クラス。
#
# @since 0.4.0
#
# @else
# @class InPortBase
# @brief Port for InPort
#
# This is an implementation class for the data input port.
#
# @since 0.4.0
#
# @endif
#
class InPortBase(OpenRTM_aist.PortBase, OpenRTM_aist.DataPortStatus):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param name ポート名称
  # @param inport 当該データ入力ポートに関連付けるInPortオブジェクト
  #               InPortオブジェクトで扱うデータ型、バッファタイプも指定する
  # @param prop ポート設定用プロパティ
  #
  # @else
  # @brief Constructor
  #
  # Constructor
  #
  # @param name Port name
  # @param inport InPort object that is associated with this data input port.
  #               Specify also the data type and the buffer type used in 
  #               the InPort object.
  # @param prop Property for setting ports
  #
  # @endif
  #
  # InPortBase(const char* name, const char* data_type);
  def __init__(self, name, data_type):
    OpenRTM_aist.PortBase.__init__(self,name)
    self._rtcout.RTC_DEBUG("Port name: %s", name)
    self._singlebuffer  = True
    self._thebuffer     = None
    self._properties    = OpenRTM_aist.Properties()
    self._providerTypes = ""
    self._consumerTypes = ""
    self._connectors    = []
    self._connector_mutex = threading.RLock()

    # PortProfile::properties を設定
    self._rtcout.RTC_DEBUG("setting port.port_type: DataInPort")
    self.addProperty("port.port_type", "DataInPort")

    self._rtcout.RTC_DEBUG("setting port.data_type: %s", data_type)
    self.addProperty("dataport.data_type", data_type)

    self.addProperty("dataport.subscription_type", "Any")
    self._value = None
    self._listeners = OpenRTM_aist.ConnectorListeners()


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
  def __del__(self, PortBase=OpenRTM_aist.PortBase):
    self._rtcout.RTC_TRACE("InPortBase destructor")

    if len(self._connectors) != 0:
      self._rtcout.RTC_ERROR("connector.size should be 0 in InPortBase's dtor.")
      # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
      for connector in self._connectors:
        connector.disconnect()

    if self._thebuffer is not None:
      OpenRTM_aist.CdrBufferFactory.instance().deleteObject(self._thebuffer)
      if not self._singlebuffer:
        self._rtcout.RTC_ERROR("Although singlebuffer flag is true, the buffer != 0")

    PortBase.__del__(self)
    return


  ##
  # @if jp
  # @brief プロパティの初期化
  #
  # 指定されたプロパティで初期化する。
  #
  # @param prop 設定するプロパティ
  # @else
  # @brief Initializing properties
  #
  # This method initializes the port in the specified property. 
  #
  # @param prop Property for setting ports
  # @endif
  #
  # void init(coil::Properties& prop);
  def init(self,prop):
    self._rtcout.RTC_TRACE("init()")
    self._properties.mergeProperties(prop)
    if self._singlebuffer:
      self._rtcout.RTC_DEBUG("single buffer mode.")
      self._thebuffer = OpenRTM_aist.CdrBufferFactory.instance().createObject("ring_buffer")

      if self._thebuffer is None:
        self._rtcout.RTC_ERROR("default buffer creation failed")
    else:
      self._rtcout.RTC_DEBUG("multi buffer mode.")

    self.initProviders()
    self.initConsumers()

    num = [-1]
    if not OpenRTM_aist.stringTo(num, self._properties.getProperty("connection_limit","-1")):
      self._rtcout.RTC_ERROR("invalid connection_limit value: %s",
                             self._properties.getProperty("connection_limit"))

    self.setConnectionLimit(num[0])
    return


  ##
  # @if jp
  # @brief RTObject_impl::readAll()から呼ばれる仮想関数
  #
  # DataPort からデータを読み出す
  #
  # @return true:成功,false:失敗
  # @else
  # @brief It is a virtual method that is called from RTObject_impl::readAll().
  # This method reads out data from DataPort. 
  #
  # @return true:Success,false:Failure
  # @endif
  #
  # virtual bool read() = 0;
  def read(self):
    pass

  ##
  # @if jp
  # @brief プロパティを取得する
  #
  # InPortのプロパティを取得する。
  #
  # @return プロパティ
  #
  # @else
  #
  # @brief Get properties
  #
  # Getting properties of this InPort
  #
  # @return InPort's properties
  #
  # @endif
  #
  def properties(self):
    self._rtcout.RTC_TRACE("properties()")
    return self._properties


  ##
  # @if jp
  # @brief Connector を取得
  #
  # 現在所有しているコネクタを取得する。
  #
  # @return connector のリスト
  #
  # @else
  #
  # @brief Connector list
  #
  # This operation returns connector list
  #
  # @return connector list
  #
  # @endif
  #
  # const std::vector<InPortConnector*>& connectors();
  def connectors(self):
    self._rtcout.RTC_TRACE("connectors(): size = %d", len(self._connectors))
    #  guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    return self._connectors


  ##
  # @if jp
  # @brief ConnectorProfile を取得
  #
  # 現在所有しているコネクタのProfileを取得する。
  #
  # @return ConnectorProfile のリスト
  #
  # @else
  #
  # @brief ConnectorProfile list
  #
  # This operation returns ConnectorProfile list
  #
  # @return connector list
  #
  # @endif
  #
  # ConnectorInfoList getConnectorProfiles();
  def getConnectorProfiles(self):
    self._rtcout.RTC_TRACE("getConnectorProfiles(): size = %d", len(self._connectors))
    profs = []
    #  guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    for con in self._connectors:
      profs.append(con.profile())

    return profs
        
  ##
  # @if jp
  # @brief ConnectorId を取得
  #
  # 現在所有しているコネクタのIDを取得する。
  #
  # @return ConnectorId のリスト
  #
  # @else
  #
  # @brief ConnectorId list
  #
  # This operation returns ConnectorId list
  #
  # @return connector list
  #
  # @endif
  #
  # coil::vstring getConnectorIds();
  def getConnectorIds(self):
    ids = []

    # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    for con in self._connectors:
      ids.append(con.id())

    self._rtcout.RTC_TRACE("getConnectorIds(): %s", OpenRTM_aist.flatten(ids))
    return ids

  ##
  # @if jp
  # @brief Connectorの名前を取得
  #
  # 現在所有しているコネクタの名前を取得する。
  #
  # @return Connector名のリスト
  #
  # @else
  #
  # @brief Connector name list
  #
  # This operation returns Connector name list
  #
  # @return connector name list
  #
  # @endif
  #
  # coil::vstring getConnectorNames();
  def getConnectorNames(self):
    names = []
    # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    for con in self._connectors:
      names.append(con.name())

    self._rtcout.RTC_TRACE("getConnectorNames(): %s", OpenRTM_aist.flatten(names))
    return names

  ##
  # @if jp
  # @brief ConnectorProfileをIDで取得
  #
  # 現在所有しているコネクタをIDで取得する。
  #
  # @param id Connector ID
  # @return コネクタへのポインタ
  #
  # @else
  #
  # @brief Getting ConnectorProfile by ID
  #
  # This operation returns Connector specified by ID.
  #
  # @param id Connector ID
  # @return A pointer to connector
  #
  # @endif
  #
  # InPortConnector* getConnectorById(const char* id);
  def getConnectorById(self, id):
    self._rtcout.RTC_TRACE("getConnectorById(id = %s)", id)

    for con in self._connectors:
      if id == con.id():
        return con

    self._rtcout.RTC_WARN("ConnectorProfile with the id(%s) not found.", id)
    return None

  ##
  # @if jp
  # @brief ConnectorProfileを名前で取得
  #
  # 現在所有しているコネクタを名前で取得する。
  #
  # @param name Connector name
  # @return コネクタへのポインタ
  #
  # @else
  #
  # @brief Getting Connector by name
  #
  # This operation returns Connector specified by name.
  #
  # @param id Connector ID
  # @return A pointer to connector
  #
  # @endif
  #
  # InPortConnector* getConnectorByName(const char* name);
  def getConnectorByName(self, name):
    self._rtcout.RTC_TRACE("getConnectorByName(name = %s)", name)

    for con in self._connectors:
      if name == con.name():
        return con

    self._rtcout.RTC_WARN("ConnectorProfile with the name(%s) not found.", name)
    return None

  ##
  # @if jp
  # @brief ConnectorProfileをIDで取得
  #
  # 現在所有しているコネクタをIDで取得する。
  #
  # @param id Connector ID
  # @param prof ConnectorProfile
  # @return false 指定したIDがない
  #
  # @else
  #
  # @brief Getting ConnectorProfile by name
  #
  # This operation returns ConnectorProfile specified by name
  #
  # @param id Connector ID
  # @param prof ConnectorProfile
  # @return false　specified ID does not exist
  #
  # @endif
  #
  # bool getConnectorProfileById(const char* id,
  #                              ConnectorInfo& prof);
  def getConnectorProfileById(self, id, prof):
    self._rtcout.RTC_TRACE("getConnectorProfileById(id = %s)", id)

    # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    conn = self.getConnectorById(id)
    if not conn:
      return False
    prof[0] = conn.profile()
    return True


  ##
  # @if jp
  # @brief ConnectorProfileを名前で取得
  #
  # 現在所有しているコネクタを名前で取得する。
  #
  # @param name Connector name
  # @param prof ConnectorProfile
  # @return false 指定した名前がない
  #
  # @else
  #
  # @brief Getting ConnectorProfile by name
  #
  # This operation returns ConnectorProfile specified by name
  #
  # @param id Connector ID
  # @param prof ConnectorProfile
  # @return false specified name does not exist
  #
  # @endif
  #
  # bool getConnectorProfileByName(const char* name,
  #                                ConnectorInfo& prof);
  def getConnectorProfileByName(self, name, prof):
    self._rtcout.RTC_TRACE("getConnectorProfileByName(name = %s)", name)

    # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    conn = self.getConnectorByName(name)
    if not conn:
      return False
    prof[0] = conn.profile()
    return True


  ##
  # @if jp
  #
  # @brief [CORBA interface] Port の接続を行う
  #
  # 与えられた ConnectoionProfile の情報に基づき、Port間の接続を確立
  # する。この関数は主にアプリケーションプログラムやツールから呼び出
  # すことを前提としている。
  # 
  # @param connector_profile ConnectorProfile
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [CORBA interface] Connect the Port
  #
  # This operation establishes connection according to the given
  # ConnectionProfile inforamtion. This function is premised on
  # calling from mainly application program or tools.
  #
  # @param connector_profile The ConnectorProfile.
  # @return ReturnCode_t The return code of ReturnCode_t type.
  #
  # @endif
  #
  def connect(self, connector_profile):
    self._rtcout.RTC_TRACE("InPortBase.connect()")
        
    if OpenRTM_aist.NVUtil.find_index(connector_profile.properties,
                                      "dataport.serializer.cdr.endian") is -1:
      self._rtcout.RTC_TRACE("ConnectorProfile dataport.serializer.cdr.endian set.")
      connector_profile.properties.append(OpenRTM_aist.NVUtil.newNV("dataport.serializer.cdr.endian","little,big"))

    return OpenRTM_aist.PortBase.connect(self, connector_profile)
        
  ##
  # @if jp
  #
  # @brief InPortを activates する
  #
  # InPortを activate する。
  #
  # @else
  #
  # @brief Activate all Port interfaces
  #
  # This operation activate all interfaces that is registered in the
  # ports.
  #
  # @endif
  #
  # void activateInterfaces();
  def activateInterfaces(self):
    self._rtcout.RTC_TRACE("activateInterfaces()")

    # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    for connector in self._connectors:
      connector.activate()
      self._rtcout.RTC_DEBUG("activate connector: %s %s",
                             (connector.name(),connector.id()))

    return


  ##
  # @if jp
  #
  # @brief 全ての Port のインターフェースを deactivates する
  #
  # Port に登録されている全てのインターフェースを deactivate する。
  #
  # @else
  #
  # @brief Deactivate all Port interfaces
  #
  # This operation deactivate all interfaces that is registered in the
  # ports.
  #
  # @endif
  #
  # void deactivateInterfaces();
  def deactivateInterfaces(self):
    self._rtcout.RTC_TRACE("deactivateInterfaces()")

    # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    for connector in self._connectors:
      connector.deactivate()
      self._rtcout.RTC_DEBUG("deactivate connector: %s %s",
                             (connector.name(),connector.id()))
    return


  ##
  # @if jp
  # @brief ConnectorDataListener リスナを追加する
  #
  # バッファ書き込みまたは読み出しイベントに関連する各種リスナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - ON_BUFFER_WRITE:          バッファ書き込み時
  # - ON_BUFFER_FULL:           バッファフル時
  # - ON_BUFFER_WRITE_TIMEOUT:  バッファ書き込みタイムアウト時
  # - ON_BUFFER_OVERWRITE:      バッファ上書き時
  # - ON_BUFFER_READ:           バッファ読み出し時
  # - ON_SEND:                  InProtへの送信時
  # - ON_RECEIVED:              InProtへの送信完了時
  # - ON_SEND_ERTIMEOUT:        OutPort側タイムアウト時
  # - ON_SEND_ERERROR:          OutPort側エラー時
  # - ON_RECEIVER_FULL:         InProt側バッファフル時
  # - ON_RECEIVER_TIMEOUT:      InProt側バッファタイムアウト時
  # - ON_RECEIVER_ERROR:        InProt側エラー時
  #
  # リスナは ConnectorDataListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # ConnectorDataListener::
  #         operator()(const ConnectorProfile&, const cdrStream&)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # OutPortに移り、OutPort解体時もしくは、
  # removeConnectorDataListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding BufferDataListener type listener
  #
  # This operation adds certain listeners related to buffer writing and
  # reading events.
  # The following listener types are available.
  #
  # - ON_BUFFER_WRITE:          At the time of buffer write
  # - ON_BUFFER_FULL:           At the time of buffer full
  # - ON_BUFFER_WRITE_TIMEOUT:  At the time of buffer write timeout
  # - ON_BUFFER_OVERWRITE:      At the time of buffer overwrite
  # - ON_BUFFER_READ:           At the time of buffer read
  # - ON_SEND:                  At the time of sending to InPort
  # - ON_RECEIVED:              At the time of finishing sending to InPort
  # - ON_SENDER_TIMEOUT:        At the time of timeout of OutPort
  # - ON_SENDER_ERROR:          At the time of error of OutPort
  # - ON_RECEIVER_FULL:         At the time of bufferfull of InPort
  # - ON_RECEIVER_TIMEOUT:      At the time of timeout of InPort
  # - ON_RECEIVER_ERROR:        At the time of error of InPort
  #
  # Listeners should have the following function operator().
  #
  # ConnectorDataListener::
  #         operator()(const ConnectorProfile&, const cdrStream&)
  #
  # The ownership of the given listener object is transferred to
  # this OutPort object in default.  The given listener object will
  # be destroied automatically in the OutPort's dtor or if the
  # listener is deleted by removeConnectorDataListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # void 
  # addConnectorDataListener(ConnectorDataListenerType type,
  #                          ConnectorDataListener* listener,
  #                          bool autoclean)
  def addConnectorDataListener(self, listener_type, listener, autoclean = True):
    self._rtcout.RTC_TRACE("addConnectorDataListener()")

    if listener_type < OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM:
      self._listeners.connectorData_[listener_type].addListener(listener, autoclean)
      return

    self._rtcout.RTC_ERROR("addConnectorDataListener(): Invalid listener type.")
    return


  ##
  # @if jp
  # @brief ConnectorDataListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing BufferDataListener type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  #
  # void removeConnectorDataListener(ConnectorDataListenerType type,
  #                                  ConnectorDataListener* listener)
  def removeConnectorDataListener(self, listener_type, listener):
    self._rtcout.RTC_TRACE("removeConnectorDataListener()")

    if listener_type < OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM:
      self._listeners.connectorData_[listener_type].removeListener(listener)
      return

    self._rtcout.RTC_ERROR("removeConnectorDataListener(): Invalid listener type.")
    return

  
  ##
  # @if jp
  # @brief ConnectorListener リスナを追加する
  #
  # バッファ書き込みまたは読み出しイベントに関連する各種リスナを設定する。
  #
  # 設定できるリスナのタイプは
  #
  # - ON_BUFFER_EMPTY:       バッファが空の場合
  # - ON_BUFFER_READTIMEOUT: バッファが空でタイムアウトした場合
  #
  # リスナは以下のシグニチャを持つ operator() を実装している必要がある。
  #
  # ConnectorListener::operator()(const ConnectorProfile&)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # OutPortに移り、OutPort解体時もしくは、
  # removeConnectorListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding ConnectorListener type listener
  #
  # This operation adds certain listeners related to buffer writing and
  # reading events.
  # The following listener types are available.
  #
  # - ON_BUFFER_EMPTY:       At the time of buffer empty
  # - ON_BUFFER_READTIMEOUT: At the time of buffer read timeout
  #
  # Listeners should have the following function operator().
  #
  # ConnectorListener::operator()(const ConnectorProfile&)
  #  
  # The ownership of the given listener object is transferred to
  # this OutPort object in default.  The given listener object will
  # be destroied automatically in the OutPort's dtor or if the
  # listener is deleted by removeConnectorListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # void addConnectorListener(ConnectorListenerType type,
  #                           ConnectorListener* listener,
  #                           bool autoclean)
  def addConnectorListener(self, listener_type, listener, autoclean = True):
    self._rtcout.RTC_TRACE("addConnectorListener()")

    if listener_type < OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM:
      self._listeners.connector_[listener_type].addListener(listener, autoclean)
      return

    self._rtcout.RTC_ERROR("addConnectorListener(): Invalid listener type.")
    return


  ##
  # @if jp
  # @brief ConnectorDataListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing BufferDataListener type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  #
  # void removeConnectorListener(ConnectorListenerType type,
  #                              ConnectorListener* listener)
  def removeConnectorListener(self, listener_type, listener):
    self._rtcout.RTC_TRACE("removeConnectorListener()")
        
    if listener_type < OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM:
      self._listeners.connector_[listener_type].removeListener(listener)
      return

    self._rtcout.RTC_ERROR("removeConnectorListener(): Invalid listener type.")
    return


  ##
  # @if jp
  # @brief Interface情報を公開する
  #
  # Interface情報を公開する。
  # 引数の ConnectorProfile に格納されている dataflow_type が push 型
  # の場合は、指定された interface_type の InPortProvider に関する情報
  # を ConnectorProfile::properties に書込み呼び出し側に戻す。
  #
  #  dataport.dataflow_type
  #
  # @param connector_profile コネクタプロファイル
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  # @brief Publish interface information
  #
  # Publish interface information.
  # Assign the Provider information that owned by this port
  # to ConnectorProfile#properties
  #
  # @param connector_profile The connector profile
  #
  # @return The return code of ReturnCode_t type
  #
  # @endif
  #
  # ReturnCode_t publishInterfaces(ConnectorProfile& connector_profile);
  def publishInterfaces(self, cprof):
    self._rtcout.RTC_TRACE("publishInterfaces()")

    retval = self._publishInterfaces()
    if retval != RTC.RTC_OK:
      return retval

    # prop: [port.outport].
    prop = copy.deepcopy(self._properties)

    conn_prop = OpenRTM_aist.Properties()
    OpenRTM_aist.NVUtil.copyToProperties(conn_prop, cprof.properties)
    prop.mergeProperties(conn_prop.getNode("dataport")) # marge ConnectorProfile

    # marge ConnectorProfile for buffer property.
    prop.mergeProperties(conn_prop.getNode("dataport.inport"))

    #
    # ここで, ConnectorProfile からの properties がマージされたため、
    # prop["dataflow_type"]: データフロータイプ
    # prop["interface_type"]: インターフェースタイプ
    # などがアクセス可能になる。
    #
    dflow_type = prop.getProperty("dataflow_type")
    dflow_type = OpenRTM_aist.normalize([dflow_type])

    if dflow_type == "push":
      self._rtcout.RTC_DEBUG("dataflow_type = push .... create PushConnector")

      # create InPortProvider
      provider = self.createProvider(cprof, prop)

      if not provider:
        self._rtcout.RTC_ERROR("InPort provider creation failed.")
        return RTC.BAD_PARAMETER

      # create InPortPushConnector
      connector = self.createConnector(cprof, prop, provider_=provider)
      if not connector:
        self._rtcout.RTC_ERROR("PushConnector creation failed.")
        return RTC.RTC_ERROR

      connector.setDataType(self._value)
      provider.setConnector(connector) # So that a provider gets endian information from a connector.

      self._rtcout.RTC_DEBUG("publishInterfaces() successfully finished.")
      return RTC.RTC_OK

    elif dflow_type == "pull":
      self._rtcout.RTC_DEBUG("dataflow_type = pull .... do nothing")
      return RTC.RTC_OK

    self._rtcout.RTC_ERROR("unsupported dataflow_type")
    return RTC.BAD_PARAMETER

    
  ##
  # @if jp
  # @brief Interfaceに接続する
  #
  # Interfaceに接続する。
  # Portが所有するConsumerに適合するProviderに関する情報を 
  # ConnectorProfile#properties から抽出し、
  # ConsumerにCORBAオブジェクト参照を設定する。
  #
  # @param connector_profile コネクタ・プロファイル
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  # @brief Subscribe to the interface
  #
  # Subscribe to interface.
  # Derive Provider information that matches Consumer owned by the Port 
  # from ConnectorProfile#properties and 
  # set the Consumer to the reference of the CORBA object.
  #
  # @param connector_profile The connector profile
  #
  # @return ReturnCode_t The return code of ReturnCode_t type
  #
  # @endif
  #
  # ReturnCode_t subscribeInterfaces(const ConnectorProfile& connector_profile);
  def subscribeInterfaces(self, cprof):
    self._rtcout.RTC_TRACE("subscribeInterfaces()")

    # prop: [port.outport].
    prop = copy.deepcopy(self._properties)
    conn_prop = OpenRTM_aist.Properties()
    OpenRTM_aist.NVUtil.copyToProperties(conn_prop, cprof.properties)
    prop.mergeProperties(conn_prop.getNode("dataport")) # marge ConnectorProfile
    prop.mergeProperties(conn_prop.getNode("dataport.inport")) # marge ConnectorProfile for buffer property.
    
    #
    # ここで, ConnectorProfile からの properties がマージされたため、
    # prop["dataflow_type"]: データフロータイプ
    # prop["interface_type"]: インターフェースタイプ
    # などがアクセス可能になる。
    #
    dflow_type = prop.getProperty("dataflow_type")
    dtype = [dflow_type]
    OpenRTM_aist.normalize(dtype)
    dflow_type = dtype[0]
    
    profile = OpenRTM_aist.ConnectorInfo(cprof.name,
                                         cprof.connector_id,
                                         OpenRTM_aist.CORBA_SeqUtil.refToVstring(cprof.ports),
                                         prop)
    if dflow_type == "push":
      self._rtcout.RTC_DEBUG("dataflow_type = push .... do nothing")

      conn = self.getConnectorById(cprof.connector_id)

      if not conn:
        self._rtcout.RTC_ERROR("specified connector not found: %s",
                               cprof.connector_id)
        return RTC.RTC_ERROR

      ret = conn.setConnectorInfo(profile)
      if ret == RTC.RTC_OK:
        self._rtcout.RTC_DEBUG("subscribeInterfaces() successfully finished.")

      return ret

    elif dflow_type == "pull":
      self._rtcout.RTC_DEBUG("dataflow_type = pull .... create PullConnector")

      # create OutPortConsumer
      consumer = self.createConsumer(cprof, prop)
      if not consumer:
        return RTC.BAD_PARAMETER

      # create InPortPullConnector
      connector = self.createConnector(cprof, prop, consumer_=consumer)
      if not connector:
        return RTC.RTC_ERROR

      ret = connector.setConnectorInfo(profile)

      if ret == RTC.RTC_OK:
        self._rtcout.RTC_DEBUG("publishInterface() successfully finished.")

      return ret

    self._rtcout.RTC_ERROR("unsupported dataflow_type")
    return RTC.BAD_PARAMETER
        
    
  ##
  # @if jp
  # @brief Interfaceへの接続を解除する
  #
  # Interfaceへの接続を解除する。
  # 与えられたConnectorProfileに関連するConsumerに設定された全てのObjectを
  # 解放し接続を解除する。
  #
  # @param connector_profile コネクタ・プロファイル
  #
  # @else
  # @brief Disconnect the interface connection
  #
  # Disconnect the interface connection.
  # Release all objects set in Consumer associated with 
  # given ConnectorProfile and unscribe the interface.
  #
  # @param connector_profile The connector profile
  #
  # @endif
  #
  # void unsubscribeInterfaces(const ConnectorProfile& connector_profile);
  def unsubscribeInterfaces(self, connector_profile):
    self._rtcout.RTC_TRACE("unsubscribeInterfaces()")
    
    id = connector_profile.connector_id
    self._rtcout.RTC_PARANOID("connector_id: %s", id)

    len_ = len(self._connectors)
    for i in range(len_):
      idx = (len_ - 1) - i
      # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
      if id == self._connectors[idx].id():
        # Connector's dtor must call disconnect()
        self._connectors[idx].deactivate()
        self._connectors[idx].disconnect()
        del self._connectors[idx]
        self._rtcout.RTC_TRACE("delete connector: %s", id)
        return

    self._rtcout.RTC_ERROR("specified connector not found: %s", id)
    return


  ##
  # @if jp
  # @brief InPort provider の初期化
  # @else
  # @brief InPort provider initialization
  # @endif
  #
  # void initProviders();
  def initProviders(self):
    self._rtcout.RTC_TRACE("initProviders()")

    # create InPort providers
    factory = OpenRTM_aist.InPortProviderFactory.instance()
    provider_types = factory.getIdentifiers()

    self._rtcout.RTC_DEBUG("available providers: %s",
                           OpenRTM_aist.flatten(provider_types))

    if self._properties.hasKey("provider_types") and \
          OpenRTM_aist.normalize(self._properties.getProperty("provider_types")) != "all":
      self._rtcout.RTC_DEBUG("allowed providers: %s",
                             self._properties.getProperty("provider_types"))

      temp_types = provider_types
      provider_types = []
      active_types = OpenRTM_aist.split(self._properties.getProperty("provider_types"), ",")

      temp_types.sort()
      active_types.sort()

      set_ptypes = set(temp_types).intersection(set(active_types))
      provider_types = provider_types + list(set_ptypes)

    # InPortProvider supports "push" dataflow type
    if len(provider_types) > 0:
      self._rtcout.RTC_DEBUG("dataflow_type push is supported")
      self.appendProperty("dataport.dataflow_type", "push")
      self.appendProperty("dataport.interface_type",
                          OpenRTM_aist.flatten(provider_types))

    self._providerTypes = provider_types
    return


  ##
  # @if jp
  # @brief OutPort consumer の初期化
  # @else
  # @brief OutPort consumer initialization
  # @endif
  #
  # void initConsumers();
  def initConsumers(self):
    self._rtcout.RTC_TRACE("initConsumers()")

    # create OuPort consumers
    factory = OpenRTM_aist.OutPortConsumerFactory.instance()
    consumer_types = factory.getIdentifiers()
    self._rtcout.RTC_DEBUG("available consumers: %s",
                           OpenRTM_aist.flatten(consumer_types))

    if self._properties.hasKey("consumer_types") and \
          OpenRTM_aist.normalize(self._properties.getProperty("consumer_types")) != "all":
      self._rtcout.RTC_DEBUG("allowed consumers: %s",
                             self._properties.getProperty("consumer_types"))

      temp_types = consumer_types
      consumer_types = []
      active_types = OpenRTM_aist.split(self._properties.getProperty("consumer_types"), ",")

      temp_types.sort()
      active_types.sort()

      set_ctypes = set(temp_types).intersection(set(active_types))
      consumer_types = consumer_types + list(set_ctypes)

    # OutPortConsumer supports "pull" dataflow type
    if len(consumer_types) > 0:
      self._rtcout.RTC_PARANOID("dataflow_type pull is supported")
      self.appendProperty("dataport.dataflow_type", "pull")
      self.appendProperty("dataport.interface_type",
                          OpenRTM_aist.flatten(consumer_types))

    self._consumerTypes = consumer_types
    return


  ##
  # @if jp
  # @brief InPort provider の生成
  #
  # InPortProvider を生成し、情報を ConnectorProfile に公開する。
  # 生成に失敗した場合 0 を返す。
  #
  # @else
  # @brief InPort provider creation
  # @endif
  #
  # InPortProvider*
  # createProvider(ConnectorProfile& cprof, coil::Properties& prop);
  def createProvider(self, cprof, prop):
    if not prop.getProperty("interface_type") and \
          not OpenRTM_aist.includes(self._providerTypes, prop.getProperty("interface_type")):
      self._rtcout.RTC_ERROR("no provider found")
      self._rtcout.RTC_DEBUG("interface_type:  %s", prop.getProperty("interface_type"))
      self._rtcout.RTC_DEBUG("interface_types: %s",
                             OpenRTM_aist.flatten(self._providerTypes))
      return 0

    
    self._rtcout.RTC_DEBUG("interface_type: %s", prop.getProperty("interface_type"))
    provider = OpenRTM_aist.InPortProviderFactory.instance().createObject(prop.getProperty("interface_type"))
    
    if provider != 0:
      self._rtcout.RTC_DEBUG("provider created")
      provider.init(prop.getNode("provider"))

      if not provider.publishInterface(cprof.properties):
        self._rtcout.RTC_ERROR("publishing interface information error")
        OpenRTM_aist.InPortProviderFactory.instance().deleteObject(provider)
        return 0
      return provider

    self._rtcout.RTC_ERROR("provider creation failed")
    return 0


  ##
  # @if jp
  # @brief OutPort consumer の生成
  #
  # OutPortConsumer を生成する。
  # 生成に失敗した場合 0 を返す。
  #
  # @else
  # @brief InPort provider creation
  # @endif
  #
  # OutPortConsumer*
  # createConsumer(const ConnectorProfile& cprof, coil::Properties& prop);
  def createConsumer(self, cprof, prop):
    if not prop.getProperty("interface_type") and \
          not OpenRTM_aist.includes(self._consumerTypes, prop.getProperty("interface_type")):
      self._rtcout.RTC_ERROR("no consumer found")
      self._rtcout.RTC_DEBUG("interface_type:  %s", prop.getProperty("interface_type"))
      self._rtcout.RTC_DEBUG("interface_types: %s",
                             OpenRTM_aist.flatten(self._consumerTypes))
      return 0
    
    self._rtcout.RTC_DEBUG("interface_type: %s", prop.getProperty("interface_type"))
    consumer = OpenRTM_aist.OutPortConsumerFactory.instance().createObject(prop.getProperty("interface_type"))
    
    if consumer != 0:
      self._rtcout.RTC_DEBUG("consumer created")
      consumer.init(prop.getNode("consumer"))

      if not consumer.subscribeInterface(cprof.properties):
        self._rtcout.RTC_ERROR("interface subscription failed.")
        OpenRTM_aist.OutPortConsumerFactory.instance().deleteObject(consumer)
        return 0
      return consumer

    self._rtcout.RTC_ERROR("consumer creation failed")
    return 0


  ##
  # @if jp
  # @brief InPortPushConnector の生成
  #
  # Connector を生成し、生成が成功すれば m_connectors に保存する。
  # 生成に失敗した場合 0 を返す。
  #
  # @else
  # @brief InPortPushConnector creation
  # @endif
  #
  # InPortConnector*
  #  createConnector(ConnectorProfile& cprof, coil::Properties& prop,
  #                  InPortProvider* provider);
  def createConnector(self, cprof, prop, provider_=None, consumer_=None):
    profile = OpenRTM_aist.ConnectorInfo(cprof.name,
                                         cprof.connector_id,
                                         OpenRTM_aist.CORBA_SeqUtil.refToVstring(cprof.ports),
                                         prop)
    connector = None


    try:
      if provider_ is not None:
        if self._singlebuffer:
          connector = OpenRTM_aist.InPortPushConnector(profile, provider_,
                                                       self._listeners,
                                                       self._thebuffer)
        else:
          connector = OpenRTM_aist.InPortPushConnector(profile, provider_,
                                                       self._listeners)

      elif consumer_ is not None:
        if self._singlebuffer:
          connector = OpenRTM_aist.InPortPullConnector(profile, consumer_,
                                                       self._listeners,
                                                       self._thebuffer)
        else:
          connector = OpenRTM_aist.InPortPullConnector(profile, consumer_,
                                                       self._listeners)

      else:
        self._rtcout.RTC_ERROR("provider or consumer is not passed. returned 0;")
        return 0
                

      if connector is None:
        self._rtcout.RTC_ERROR("InPortConnector creation failed")
        return 0

      if provider_ is not None:
        self._rtcout.RTC_TRACE("InPortPushConnector created")
      elif consumer_ is not None:
        self._rtcout.RTC_TRACE("InPortPullConnector created")

      # guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
      self._connectors.append(connector)
      self._rtcout.RTC_PARANOID("connector push backed: %d", len(self._connectors))
      return connector
    except:
      self._rtcout.RTC_ERROR("InPortPushConnector creation failed")
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      return 0

    self._rtcout.RTC_FATAL("never comes here: createConnector()")
    return 0
