#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file OutPortBase.py
# @brief OutPortBase base class
# @date $Date: 2007/09/19 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import copy
import threading
import OpenRTM_aist
import RTC

##
# @if jp
#
# @class OutPortBase
#
# @brief OutPort 基底クラス
# 
# OutPort の基底クラス。
#
#
#
# Properties: port.outport
# プロパティは
#
# - port.outport
# - port.outport.[name]
# ConnectorProfile.properties の場合は
# - dataport.outport
#
# 以下に指定したものが渡される。
# (port.outport.[name]が優先される)
# さらに、一部のプロパティは接続時に ConnectorProfile により
# 渡される場合があり、その場合は ConnectorProfile が優先される。
#
# - input.throughput.profile: enable
# - input.throughput.update_rate: count [n/count]
# - input.throughput.total_bytes: [bytes]
# - input.throughput.total_count: [n]
# - input.throughput.max_size: [bytes]
# - input.throughput.min_size: [bytes]
# - input.throughput.avg_size: [bytes]
# - input.throughput.byte_sec: [bytes/sec]
#
# - output.throughput.profile: enable
# - output.throughput.update_rate: count [n/count]
# - output.throughput.total_bytes: [bytes]
# - output.throughput.total_count:[n]
# - output.throughput.max_size: [bytes]
# - output.throughput.min_size: [bytes]
# - output.throughput.avg_size: [bytes]
# - output.throughput.max_sendtime: [sec]
# - output.throughput.min_sendtime: [sec]
# - output.throughput.avg_sendtime: [sec]
# - output.throughput.byte_sec: [bytes/sec]
#
# dataport.dataflow_type
# dataport.interface_type
# dataport.subscription_type
#
# [buffer]
#
# - buffer.type:
#     利用可能なバッファのタイプ
#     ConnectorProfile の場合は利用するバッファのタイプ
#     無指定の場合はデフォルトの ringbuffer が使用される。
#     ex. ringbuffer, shmbuffer, doublebuffer, etc.
#     正し、Consumer, Publisher のタイプによっては特定のバッファ型を
#     要求するがあるための、その場合は指定は無効となる。
#
# - buffer.length:
#     バッファの長さ
#
# - buffer.write.full_policy:
#     上書きするかどうかのポリシー
#     overwrite (上書き), do_nothing (何もしない), block (ブロックする)
#     block を指定した場合、次の timeout 値を指定すれば、指定時間後
#     書き込み不可能であればタイムアウトする。
#
# - buffer.write.timeout:
#     タイムアウト時間を [sec] で指定する。
#     1 sec -> 1.0, 1 ms -> 0.001, タイムアウトしない -> 0.0
#
# - buffer.read.empty_policy:
#     バッファが空のときの読み出しポリシー
#     last (最後の要素), do_nothing (何もしない), block (ブロックする)
#     block を指定した場合、次の timeout 値を指定すれば、指定時間後
#     読み出し不可能であればタイムアウトする。
#
# - buffer.read.timeout:
#     タイムアウト時間 [sec] で指定する。
#     1sec -> 1.0, 1ms -> 0.001, タイムアウトしない -> 0.0
#
# - その他バッファ毎に固有なオプション
#
#
# [publihser]
#
# - publisher.types:
#      利用可能な Publisher のタイプ
#      new, periodic, flush, etc..
#
# - publisher.push.policy:
#      InPortへデータを送信するポリシー
#      all: バッファにたまっているすべて送信
#      fifo: バッファをFIFOとみなして送信
#      skip: 古いデータから一定数を間引いて送信
#      new: 常に新しいデータのみを送信
#
# - publisher.push.skip_rate:
#      push.policy=skip のときのスキップ率
#      n: n要素毎にひとつ送信
#
# - publisher.periodic.rate:
#
# - publisher.thread.type:
#       Publisher のスレッドのタイプ
# - publisher.thread.measurement.exec_time: yes/no
# - publisher.thread.measurement.exec_count: number
# - publisher.thread.measurement.period_time: yes/no
# - publisher.thread.measurement.period_count: number
#
# [interface]
#
# - interface.types:
#     OutPort interfaceのタイプ
#     ex. corba_cdr, corba_any, raw_tcp などカンマ区切りで指定。何も
#     指定しなければ利用可能なすべてのプロバイダが使用される
#
#
#
#   
# OutPort 側の connect() では以下のシーケンスで処理が行われる。
#
# 1. OutPort に関連する connector 情報の生成およびセット
#
# 2. InPortに関連する connector 情報の取得
#  - ConnectorProfile::properties["dataport.corba_any.inport_ref"]に
#    OutPortAny のオブジェクトリファレンスが設定されている場合、
#    リファレンスを取得してConsumerオブジェクトにセットする。
#    リファレンスがセットされていなければ無視して継続。
#    (OutPortがconnect() 呼び出しのエントリポイントの場合は、
#    InPortのオブジェクトリファレンスはセットされていないはずである。)
# 3. PortBase::connect() をコール
#    Portの接続の基本処理が行われる。
# 4. 上記2.でInPortのリファレンスが取得できなければ、再度InPortに
#    関連する connector 情報を取得する。
#
# 5. ConnectorProfile::properties で与えられた情報から、
#    OutPort側の初期化処理を行う。
#
# - [dataport.interface_type]
# -- CORBA_Any の場合: 
#    InPortAny を通してデータ交換される。
#    ConnectorProfile::properties["dataport.corba_any.inport_ref"]に
#    InPortAny のオブジェクトリファレンスをセットする。
# -- RawTCP の場合: Raw TCP socket を通してデータ交換される。
#    ConnectorProfile::properties["dataport.raw_tcp.server_addr"]
#    にInPort側のサーバアドレスをセットする。
#
# - [dataport.dataflow_type]
# -- Pushの場合: Subscriberを生成する。Subscriberのタイプは、
#    dataport.subscription_type に設定されている。
# -- Pullの場合: InPort側がデータをPull型で取得するため、
#    特に何もする必要が無い。
#
# - [dataport.subscription_type]
# -- Onceの場合: SubscriberOnceを生成する。
# -- Newの場合: SubscriberNewを生成する。
# -- Periodicの場合: SubscriberPeriodicを生成する。
#
# - [dataport.push_interval]
# -- dataport.subscription_type=Periodicの場合周期を設定する。
#
# 6. 上記の処理のうち一つでもエラーであれば、エラーリターンする。
#    正常に処理が行われた場合はRTC::RTC_OKでリターンする。
#
# @since 0.2.0
#
# @else
#
# @class OutPortBase
#
# @brief Output base class.
#
# The base class of OutPort<T> which are implementations of OutPort
#
# Form a kind of Observer pattern with OutPortBase and PublisherBase.
# attach(), detach(), notify() of OutPortBase and
# push() of PublisherBase are methods associated with the Observer pattern.
#
# @since 0.2.0
#
# @endif
#
class OutPortBase(OpenRTM_aist.PortBase,OpenRTM_aist.DataPortStatus):
  """
  """

  ##
  # @if jp
  # @brief Provider を削除するための Functor
  # @else
  # @brief Functor to delete Providers
  # @endif
  #
  class provider_cleanup:
    def __init__(self):
      self._factory = OpenRTM_aist.OutPortProviderFactory.instance()

    def __call__(self, p):
      self._factory.deleteObject(p)

  ##
  # @if jp
  # @brief Connector を削除するための Functor
  # @else
  # @brief Functor to delete Connectors
  # @endif
  #
  class connector_cleanup:
    def __init__(self):
      pass

    def __call__(self, c):
      del c


  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ。
  #
  # @param self
  # @param name ポート名
  #
  # @else
  #
  # @brief A constructor of OutPortBase class.
  #
  # Constructor of OutPortBase.
  #
  # @endif
  # OutPortBase::OutPortBase(const char* name, const char* data_type)
  def __init__(self, name, data_type):
    OpenRTM_aist.PortBase.__init__(self,name)
    self._rtcout.RTC_DEBUG("Port name: %s", name)

    self._rtcout.RTC_DEBUG("setting port.port_type: DataOutPort")
    self.addProperty("port.port_type", "DataOutPort")

    self._rtcout.RTC_DEBUG("setting dataport.data_type: %s", data_type)
    self.addProperty("dataport.data_type", data_type)

    # publisher list
    factory = OpenRTM_aist.PublisherFactory.instance()
    pubs = OpenRTM_aist.flatten(factory.getIdentifiers())

    # blank characters are deleted for RTSE's bug
    pubs = pubs.lstrip()

    self._rtcout.RTC_DEBUG("available subscription_type: %s",  pubs)
    self.addProperty("dataport.subscription_type", pubs)

    self._properties    = OpenRTM_aist.Properties()
    self._name          = name
    self._connectors    = []
    self._consumers     = []
    self._providerTypes = ""
    self._consumerTypes = ""
    self._connector_mutex = threading.RLock()

    self._listeners = OpenRTM_aist.ConnectorListeners()
    return


  ##
  # @if jp
  # @brief デストラクタ
  #
  # デストラクタ。
  # 登録された全ての Publisher を削除する。
  #
  # @param self
  #
  # @else
  #
  # @brief destructor
  #
  # Destructor
  #
  # @endif
  def __del__(self, PortBase=OpenRTM_aist.PortBase):
    self._rtcout.RTC_TRACE("OutPortBase destructor")
    # connector のクリーンナップ
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._connectors,
                                        self.connector_cleanup())
    PortBase.__del__(self)
    return


  ##
  # @if jp
  # @brief プロパティの初期化
  #
  # OutPortのプロパティを初期化する
  #
  # @else
  #
  # @brief Initializing properties
  #
  # This operation initializes outport's properties
  #
  # @endif
  #
  # void init(coil::Properties& prop);
  def init(self, prop):
    self._rtcout.RTC_TRACE("init()")

    self._properties.mergeProperties(prop)

    self.configure()

    self.initConsumers()
    self.initProviders()

    num = [-1]
    if not OpenRTM_aist.stringTo(num, self._properties.getProperty("connection_limit","-1")):
      self._rtcout.RTC_ERROR("invalid connection_limit value: %s",
                             self._properties.getProperty("connection_limit"))

    self.setConnectionLimit(num[0])
    return

  ##
  # @if jp
  #
  # @brief データ書き込み
  #
  # ポートへデータを書き込む。
  # バインドされた変数に設定された値をポートに書き込む。
  #
  # @return 書き込み処理結果(書き込み成功:true、書き込み失敗:false)
  #
  # @else
  #
  # @brief Write data
  #
  # Write data to the port.
  # Write the value, which was set to the bound variable, to the port.
  #
  # @return Writing result (Successful:true, Failed:false)
  #
  # @endif
  #
  # virtual bool write() = 0;
  def write(self):
    pass


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
    self._rtcout.RTC_TRACE("OutPortBase.connect()")
        
    if OpenRTM_aist.NVUtil.find_index(connector_profile.properties,
                                      "dataport.serializer.cdr.endian") is -1:
      self._rtcout.RTC_TRACE("ConnectorProfile dataport.serializer.cdr.endian set.")
      connector_profile.properties.append(OpenRTM_aist.NVUtil.newNV("dataport.serializer.cdr.endian","little,big"))

    return OpenRTM_aist.PortBase.connect(self, connector_profile)
        

  ##
  # @if jp
  # @brief プロパティを取得する
  #
  # OutPortのプロパティを取得する。
  #
  # @return プロパティ
  #
  # @else
  #
  # @brief Get properties
  #
  # Getting properties of this OutPort
  #
  # @return OutPort's properties
  #
  # @endif
  #
  # coil::Properties& OutPortBase::properties()
  def properties(self):
    self._rtcout.RTC_TRACE("properties()")
    return self._properties


  ##
  # @if jp
  # @brief Connector を取得
  # @else
  # @brief Connector list
  # @endif
  #
  # const std::vector<OutPortConnector*>& OutPortBase::connectors()
  def connectors(self):
    self._rtcout.RTC_TRACE("connectors(): size = %d", len(self._connectors))
    return self._connectors


  ##
  # @if jp
  # @brief ConnectorProfile を取得
  # @else
  # @brief ConnectorProfile list
  # @endif
  #
  # ConnectorBase::ConnectorInfoList OutPortBase::getConnectorProfiles()
  def getConnectorProfiles(self):
    self._rtcout.RTC_TRACE("getConnectorProfiles(): size = %d", len(self._connectors))
    profs = []
    for con in self._connectors:
      profs.append(con.profile())

    return profs


  ##
  # @if jp
  # @brief ConnectorId を取得
  # @else
  # @brief ConnectorId list
  # @endif
  #
  # coil::vstring OutPortBase::getConnectorIds()
  def getConnectorIds(self):
    ids = []

    for con in self._connectors:
      ids.append(con.id())

    self._rtcout.RTC_TRACE("getConnectorIds(): %s", OpenRTM_aist.flatten(ids))
    return ids


  ##
  # @if jp
  # @brief Connectorの名前を取得
  # @else
  # @brief Connector name list
  # @endif
  #
  # coil::vstring OutPortBase::getConnectorNames()
  def getConnectorNames(self):
    names = []
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
  # OutPortConnector* getConnectorById(const char* id);
  def getConnectorById(self, id):
    self._rtcout.RTC_TRACE("getConnectorById(id = %s)", id)

    for (i,con) in enumerate(self._connectors):
      if id == con.id():
        return self._connectors[i]

    self._rtcout.RTC_WARN("ConnectorProfile with the id(%s) not found.", id)
    return 0

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
  # OutPortConnector* getConnectorByName(const char* name);
  def getConnectorByName(self, name):
    self._rtcout.RTC_TRACE("getConnectorByName(name = %s)", name)
    
    for (i,con) in enumerate(self._connectors):
      if name == con.name():
        return self._connectors[i]

    self._rtcout.RTC_WARN("ConnectorProfile with the name(%s) not found.", name)
    return 0


  ##
  # @if jp
  # @brief ConnectorProfileをIDで取得
  # @else
  # @brief Getting ConnectorProfile by name
  # @endif
  #
  # bool OutPortBase::getConnectorProfileById(const char* id,
  #                                           ConnectorInfo& prof)
  def getConnectorProfileById(self, id, prof):
    self._rtcout.RTC_TRACE("getConnectorProfileById(id = %s)", id)

    conn = self.getConnectorById(id)

    if not conn:
      return False

    prof[0] = conn.profile()
    return True


  ##
  # @if jp
  # @brief ConnectorProfileを名前で取得
  # @else
  # @brief Getting ConnectorProfile by name
  # @endif
  #
  # bool OutPortBase::getConnectorProfileByName(const char* name,
  #                                             ConnectorInfo& prof)
  def getConnectorProfileByName(self, name, prof):
    self._rtcout.RTC_TRACE("getConnectorProfileByName(name = %s)", name)

    conn = self.getConnectorByName(name)

    if not conn:
      return False

    prof[0] = conn.profile()
    return True


  ##
  # @if jp
  # @brief OutPortを activates する
  # @else
  # @brief Activate all Port interfaces
  # @endif
  #
  # void OutPortBase::activateInterfaces()
  def activateInterfaces(self):
    self._rtcout.RTC_TRACE("activateInterfaces()")
    for con in self._connectors:
      con.activate()

  
  ##
  # @if jp
  # @brief 全ての Port のインターフェースを deactivates する
  # @else
  # @brief Deactivate all Port interfaces
  # @endif
  #
  # void OutPortBase::deactivateInterfaces()
  def deactivateInterfaces(self):
    self._rtcout.RTC_TRACE("deactivateInterfaces()")
    for con in self._connectors:
      con.deactivate()
  

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
  # void addConnectorDataListener(ConnectorDataListenerType listener_type,
  #                               ConnectorDataListener* listener,
  #                               bool autoclean = true);
  def addConnectorDataListener(self, listener_type, listener, autoclean = True):
    self._rtcout.RTC_TRACE("addConnectorDataListener()")
    if listener_type < OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM:
      self._rtcout.RTC_TRACE("addConnectorDataListener(%s)",
                             OpenRTM_aist.ConnectorDataListener.toString(listener_type))
      self._listeners.connectorData_[listener_type].addListener(listener, autoclean)
      return

    self._rtcout.RTC_ERROR("addConnectorDataListener(): Unknown Listener Type")
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
  # void removeConnectorDataListener(ConnectorDataListenerType listener_type,
  #                                  ConnectorDataListener* listener);
  def removeConnectorDataListener(self, listener_type, listener):
    self._rtcout.RTC_TRACE("removeConnectorDataListener()")

    if listener_type < OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM:
      self._rtcout.RTC_TRACE("removeConnectorDataListener(%s)",
                             OpenRTM_aist.ConnectorDataListener.toString(listener_type))
      self._listeners.connectorData_[listener_type].removeListener(listener)
      return

    self._rtcout.RTC_ERROR("removeConnectorDataListener(): Unknown Listener Type")
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
  # void addConnectorListener(ConnectorListenerType callback_type,
  #                           ConnectorListener* listener,
  #                           bool autoclean = true);
  def addConnectorListener(self, callback_type, listener, autoclean = True):
    self._rtcout.RTC_TRACE("addConnectorListener()")

    if callback_type < OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM:
      self._rtcout.RTC_TRACE("addConnectorListener(%s)",
                             OpenRTM_aist.ConnectorListener.toString(callback_type))
      self._listeners.connector_[callback_type].addListener(listener, autoclean)
      return
    self._rtcout.RTC_ERROR("addConnectorListener(): Unknown Listener Type")
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
  # void removeConnectorListener(ConnectorListenerType callback_type,
  #                              ConnectorListener* listener);
  def removeConnectorListener(self, callback_type, listener):
    self._rtcout.RTC_TRACE("removeConnectorListener()")
        
    if callback_type < OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM:
      self._rtcout.RTC_TRACE("removeConnectorListener(%s)",
                             OpenRTM_aist.ConnectorListener.toString(callback_type))
      self._listeners.connector_[callback_type].removeListener(listener)
      return
    self._rtcout.RTC_ERROR("removeConnectorListener(): Unknown Listener Type")
    return


  ##
  # @if jp
  # @brief OutPortの設定を行う
  # @else
  # @brief Configureing outport
  # @endif
  #
  #void OutPortBase::configure()
  def configure(self):
    pass


  ##
  # @if jp
  # @brief Interface情報を公開する
  # @else
  # @brief Publish interface information
  # @endif
  #
  # ReturnCode_t OutPortBase::publishInterfaces(ConnectorProfile& cprof)
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

    """
    #  marge ConnectorProfile for buffer property.
    #  e.g.
    #      prof[buffer.write.full_policy]
    #           << cprof[dataport.outport.buffer.write.full_policy]
    #    
    """
    prop.mergeProperties(conn_prop.getNode("dataport.outport"))


    #
    # ここで, ConnectorProfile からの properties がマージされたため、
    # prop["dataflow_type"]: データフロータイプ
    # prop["interface_type"]: インターフェースタイプ
    # などがアクセス可能になる。
    dflow_type = OpenRTM_aist.normalize([prop.getProperty("dataflow_type")])

    if dflow_type == "push":
      self._rtcout.RTC_PARANOID("dataflow_type = push .... do nothing")
      return RTC.RTC_OK

    elif dflow_type == "pull":
      self._rtcout.RTC_PARANOID("dataflow_type = pull .... create PullConnector")

      provider = self.createProvider(cprof, prop)
      if not provider:
        return RTC.BAD_PARAMETER
        
      # create InPortPushConnector
      connector = self.createConnector(cprof, prop, provider_ = provider)
      if not connector:
        return RTC.RTC_ERROR

      # connector set
      provider.setConnector(connector)

      self._rtcout.RTC_DEBUG("publishInterface() successfully finished.")
      return RTC.RTC_OK

    self._rtcout.RTC_ERROR("unsupported dataflow_type")

    return RTC.BAD_PARAMETER


  ##
  # @if jp
  # @brief Interface情報を取得する
  # @else
  # @brief Subscribe interface
  # @endif
  #
  # ReturnCode_t OutPortBase::subscribeInterfaces(const ConnectorProfile& cprof)
  def subscribeInterfaces(self, cprof):
    self._rtcout.RTC_TRACE("subscribeInterfaces()")

    # prop: [port.outport].
    prop = copy.deepcopy(self._properties)

    conn_prop = OpenRTM_aist.Properties()
    OpenRTM_aist.NVUtil.copyToProperties(conn_prop, cprof.properties)
    prop.mergeProperties(conn_prop.getNode("dataport")) # marge ConnectorProfile
    """
    #  marge ConnectorProfile for buffer property.
    #   e.g.
    #     prof[buffer.write.full_policy]
    #          << cprof[dataport.outport.buffer.write.full_policy]
    """
    prop.mergeProperties(conn_prop.getNode("dataport.outport"))

    #
    # ここで, ConnectorProfile からの properties がマージされたため、
    # prop["dataflow_type"]: データフロータイプ
    # prop["interface_type"]: インターフェースタイプ
    # などがアクセス可能になる。
    #
    dflow_type = OpenRTM_aist.normalize([prop.getProperty("dataflow_type")])
    
    profile = OpenRTM_aist.ConnectorInfo(cprof.name,
                                         cprof.connector_id,
                                         OpenRTM_aist.CORBA_SeqUtil.refToVstring(cprof.ports),
                                         prop)
    if dflow_type == "push":
      self._rtcout.RTC_PARANOID("dataflow_type = push .... create PushConnector")

      # interface
      consumer = self.createConsumer(cprof, prop)
      if not consumer:
        return RTC.BAD_PARAMETER

      # create OutPortPushConnector
      connector = self.createConnector(cprof, prop, consumer_ = consumer)
      if not connector:
        return RTC.RTC_ERROR

      ret = connector.setConnectorInfo(profile)

      if ret == RTC.RTC_OK:
        self._rtcout.RTC_DEBUG("subscribeInterfaces() successfully finished.")

      return ret

    elif dflow_type == "pull":
      self._rtcout.RTC_PARANOID("dataflow_type = pull.")

      conn = self.getConnectorById(cprof.connector_id)
      if not conn:
        self._rtcout.RTC_ERROR("specified connector not found: %s",
                               cprof.connector_id)
        return RTC.RTC_ERROR

      ret = conn.setConnectorInfo(profile)

      if ret == RTC.RTC_OK:
        self._rtcout.RTC_DEBUG("subscribeInterfaces() successfully finished.")

      return ret

    self._rtcout.RTC_ERROR("unsupported dataflow_type")
    return RTC.BAD_PARAMETER


  ##
  # @if jp
  # @brief 登録されているInterface情報を解除する
  # @else
  # @brief Unsubscribe interface
  # @endif
  #
  # void
  # OutPortBase::unsubscribeInterfaces(const ConnectorProfile& connector_profile)
  def unsubscribeInterfaces(self, connector_profile):
    self._rtcout.RTC_TRACE("unsubscribeInterfaces()")

    id = connector_profile.connector_id
    self._rtcout.RTC_PARANOID("connector_id: %s", id)

    len_ = len(self._connectors)
    for i in range(len_):
      idx = (len_ - 1) - i
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
  # @brief OutPort provider の初期化
  # @else
  # @brief OutPort provider initialization
  # @endif
  #
  # void OutPortBase::initProviders()
  def initProviders(self):
    self._rtcout.RTC_TRACE("initProviders()")

    # create OutPort providers
    factory = OpenRTM_aist.OutPortProviderFactory.instance()
    provider_types = factory.getIdentifiers()
    self._rtcout.RTC_PARANOID("available OutPortProviders: %s",
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

    # OutPortProvider supports "pull" dataflow type
    if len(provider_types) > 0:
      self._rtcout.RTC_DEBUG("dataflow_type pull is supported")
      self.appendProperty("dataport.dataflow_type", "pull")
      self.appendProperty("dataport.interface_type",
                          OpenRTM_aist.flatten(provider_types))

    self._providerTypes = provider_types


  ##
  # @if jp
  # @brief InPort consumer の初期化
  # @else
  # @brief InPort consumer initialization
  # @endif
  #
  # void OutPortBase::initConsumers()
  def initConsumers(self):
    self._rtcout.RTC_TRACE("initConsumers()")

    # create InPort consumers
    factory = OpenRTM_aist.InPortConsumerFactory.instance()
    consumer_types = factory.getIdentifiers()
    self._rtcout.RTC_PARANOID("available InPortConsumer: %s",
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

    # InPortConsumer supports "push" dataflow type
    if len(consumer_types) > 0:
      self._rtcout.RTC_PARANOID("dataflow_type push is supported")
      self.appendProperty("dataport.dataflow_type", "push")
      self.appendProperty("dataport.interface_type",
                          OpenRTM_aist.flatten(consumer_types))
    
    self._consumerTypes = consumer_types


  ##
  # @if jp
  # @brief OutPort provider の生成
  # @else
  # @brief OutPort provider creation
  # @endif
  #
  # OutPortProvider*
  # OutPortBase::createProvider(ConnectorProfile& cprof, coil::Properties& prop)
  def createProvider(self, cprof, prop):
    if prop.getProperty("interface_type") and \
          not OpenRTM_aist.includes(self._providerTypes, prop.getProperty("interface_type")):
      self._rtcout.RTC_ERROR("no provider found")
      self._rtcout.RTC_DEBUG("interface_type:  %s", prop.getProperty("interface_type"))
      self._rtcout.RTC_DEBUG("interface_types: %s",
                             OpenRTM_aist.flatten(self._providerTypes))
      return 0

    self._rtcout.RTC_DEBUG("interface_type: %s", prop.getProperty("interface_type"))
    provider = OpenRTM_aist.OutPortProviderFactory.instance().createObject(prop.getProperty("interface_type"))
    
    if provider != 0:
      self._rtcout.RTC_DEBUG("provider created")
      provider.init(prop.getNode("provider"))

      if not provider.publishInterface(cprof.properties):
        self._rtcout.RTC_ERROR("publishing interface information error")
        OpenRTM_aist.OutPortProviderFactory.instance().deleteObject(provider)
        return 0

      return provider

    self._rtcout.RTC_ERROR("provider creation failed")
    return 0


  ##
  # @if jp
  # @brief InPort consumer の生成
  # @else
  # @brief InPort consumer creation
  # @endif
  #
  # InPortConsumer* OutPortBase::createConsumer(const ConnectorProfile& cprof,
  #                                             coil::Properties& prop)
  def createConsumer(self, cprof, prop):
    if prop.getProperty("interface_type") and \
          not self._consumerTypes.count(prop.getProperty("interface_type")):
      self._rtcout.RTC_ERROR("no consumer found")
      self._rtcout.RTC_DEBUG("interface_type:  %s", prop.getProperty("interface_type"))
      self._rtcout.RTC_DEBUG("interface_types: %s",
                             OpenRTM_aist.flatten(self._consumerTypes))
      return 0
    
    self._rtcout.RTC_DEBUG("interface_type: %s", prop.getProperty("interface_type"))
    consumer = OpenRTM_aist.InPortConsumerFactory.instance().createObject(prop.getProperty("interface_type"))
    
    if consumer != 0:
      self._rtcout.RTC_DEBUG("consumer created")
      consumer.init(prop.getNode("consumer"))

      if not consumer.subscribeInterface(cprof.properties):
        self._rtcout.RTC_ERROR("interface subscription failed.")
        OpenRTM_aist.InPortConsumerFactory.instance().deleteObject(consumer)
        return 0

      return consumer

    self._rtcout.RTC_ERROR("consumer creation failed")
    return 0


  ##
  # @if jp
  # @brief OutPortPushConnector の生成
  # @else
  # @brief OutPortPushConnector creation
  # @endif
  #
  # OutPortConnector*
  # OutPortBase::createConnector(const ConnectorProfile& cprof,
  #                              coil::Properties& prop,
  #                              InPortConsumer* consumer)
  def createConnector(self, cprof, prop, provider_ = None, consumer_ = None):
    profile = OpenRTM_aist.ConnectorInfo(cprof.name,
                                         cprof.connector_id,
                                         OpenRTM_aist.CORBA_SeqUtil.refToVstring(cprof.ports),
                                         prop)

    connector = None
    try:

      if consumer_ is not None:
        connector = OpenRTM_aist.OutPortPushConnector(profile, consumer_, self._listeners)
      elif provider_ is not None:
        connector = OpenRTM_aist.OutPortPullConnector(profile, provider_, self._listeners)

      else:
        self._rtcout.RTC_ERROR("provider or consumer is not passed. returned 0;")
        return 0

      if connector is None:
        self._rtcout.RTC_ERROR("OutPortConnector creation failed")
        return 0

      if consumer_ is not None:
        self._rtcout.RTC_TRACE("OutPortPushConnector created")
      elif provider_ is not None:
        self._rtcout.RTC_TRACE("OutPortPullConnector created")

      self._connectors.append(connector)
      self._rtcout.RTC_PARANOID("connector push backed: %d", len(self._connectors))
      return connector

    except:
      self._rtcout.RTC_ERROR("Exeption: OutPortPushConnector creation failed")
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      return 0


    self._rtcout.RTC_FATAL("never comes here: createConnector()")
    return 0
