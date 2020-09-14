#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file PortBase.py
# @brief RTC's Port base class
# @date $Date: 2007/09/18 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import threading
import copy

import OpenRTM_aist
import RTC, RTC__POA



##
# @if jp
# @class PortBase
# @brief Port の基底クラス
#
# RTC::Port の基底となるクラス。
# RTC::Port はほぼ UML Port の概念を継承しており、ほぼ同等のものとみなす
# ことができる。RT コンポーネントのコンセプトにおいては、
# Port はコンポーネントに付属し、コンポーネントが他のコンポーネントと相互作用
# を行う接点であり、通常幾つかのインターフェースと関連付けられる。
# コンポーネントは Port を通して外部に対しインターフェースを提供または要求
# することができ、Portはその接続を管理する役割を担う。
# <p>
# Port の具象クラスは、通常 RT コンポーネントインスタンス生成時に同時に
# 生成され、提供・要求インターフェースを登録した後、RT コンポーネントに
# 登録され、外部からアクセス可能な Port として機能することを想定している。
# <p>
# RTC::Port は CORBA インターフェースとして以下のオペレーションを提供する。
#
# - get_port_profile()
# - get_connector_profiles()
# - get_connector_profile()
# - connect()
# - notify_connect()
# - disconnect()
# - notify_disconnect()
# - disconnect_all()
#
# このクラスでは、これらのオペレーションの実装を提供する。
# <p>
# これらのオペレーションのうち、get_port_profile(), get_connector_profiles(),
# get_connector_profile(), connect(), disconnect(), disconnect_all() は、
# サブクラスにおいて特に振る舞いを変更する必要がないため、オーバーライド
# することは推奨されない。
# <p>
# notify_connect(), notify_disconnect() については、サブクラスが提供・要求
# するインターフェースの種類に応じて、振る舞いを変更する必要が生ずる
# かもしれないが、これらを直接オーバーライドすることは推奨されず、
# 後述の notify_connect(), notify_disconnect() の項においても述べられる通り
# これらの関数に関連した 関数をオーバーライドすることにより振る舞いを変更する
# ことが推奨される。
#
# @since 0.4.0
#
# @else
# @class PortBase
# @brief Port base class
#
# This class is a base class of RTC::Port.
# RTC::Port inherits a concept of RT-Component, and can be regarded as almost
# the same as it. In the concept of RT-Component, Port is attached to the
# component, can mediate interaction between other components and usually is
# associated with some interfaces.
# Component can provide or require interface for outside via Port, and the
# Port plays a role to manage the connection.
# <p>
# Concrete class of Port assumes to be usually created at the same time that
# RT-Component's instance is created, be registerd to RT-Component after
# provided and required interfaces are registerd, and function as accessible
# Port from outside.
# <p>
# RTC::Port provides the following operations as CORBA interface:
#
# - get_port_profile()
# - get_connector_profiles()
# - get_connector_profile()
# - connect()
# - notify_connect()
# - disconnect()
# - notify_disconnect()
# - disconnect_all()
#
# This class provides implementations of these operations.
# <p>
# In these operations, as for get_port_profile(), get_connector_profiles(),
# get_connector_profile(), connect(), disconnect() and disconnect_all(),
# since their behaviors especially need not to be change in subclass, 
# overriding is not recommended.
# <p>
# As for notify_connect() and notify_disconnect(), you may have to modify
# behavior according to the kind of interfaces that subclass provides and
# requires, however it is not recommended these are overriden directly.
# In the section of notify_connect() and notify_disconnect() as described
# below, it is recommended that you modify behavior by overriding the
# protected function related to these functions.
#
# @since 0.4.0
#
# @endif
class PortBase(RTC__POA.PortService):
  """
  """



  ##
  # @if jp
  # @brief コンストラクタ
  #
  # PortBase のコンストラクタは Port 名 name を引数に取り初期化を行う
  # と同時に、自分自身を CORBA Object として活性化し、自身の PortProfile
  # の port_ref に自身のオブジェクトリファレンスを格納する。
  # 名前には、"." 以外の文字列を使用することができる。
  #
  # @param self
  # @param name Port の名前(デフォルト値:None)
  #
  # @else
  #
  # @brief Constructor
  #
  # The constructor of the ProtBase class is given the name of this Port
  # and initialized. At the same time, the PortBase activates itself
  # as CORBA object and stores its object reference to the PortProfile's 
  # port_ref member.
  # Characters except "." can be used for the name of the port.
  #
  # @param name The name of Port 
  #
  # @endif
  def __init__(self, name=None):
    self._ownerInstanceName = "unknown"
    self._objref = self._this()

    self._profile = RTC.PortProfile("", [], RTC.PortService._nil, [], RTC.RTObject._nil,[])
    # Now Port name is <instance_name>.<port_name>. r1648
    if name is None:
      self._profile.name = "unknown.unknown"
    else:
      self._profile.name = self._ownerInstanceName+"."+name
      
    self._profile.port_ref = self._objref
    self._profile.owner = RTC.RTObject._nil
    self._profile_mutex = threading.RLock()
    self._connection_mutex = threading.RLock()
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf(name)
    self._onPublishInterfaces = None
    self._onSubscribeInterfaces = None
    self._onConnected = None
    self._onUnsubscribeInterfaces = None
    self._onDisconnected = None
    self._onConnectionLost = None
    self._connectionLimit   = -1
    self._portconnListeners = None
    return

  
  ##
  # @if jp
  #
  # @brief デストラクタ
  #
  # デストラクタでは、PortService CORBA オブジェクトの deactivate を
  # 行う。deactivateに際して例外を投げることはない。
  #
  # @else
  #
  # @brief Destructor
  #
  # In the destructor, PortService CORBA object is deactivated.
  # This function never throws exception.
  #
  # @endif
  #
  def __del__(self):
    self._rtcout.RTC_TRACE("PortBase.__del__()")
    try:
      mgr = OpenRTM_aist.Manager.instance().getPOA()
      oid = mgr.servant_to_id(self)
      mgr.deactivate_object(oid)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
    

  ##
  # @if jp
  #
  # @brief [CORBA interface] PortProfileを取得する
  #
  # Portが保持するPortProfileを返す。
  # PortProfile 構造体は以下のメンバーを持つ。
  #
  # - name              [string 型] Port の名前。
  # - interfaces        [PortInterfaceProfileList 型] Port が保持する
  #                     PortInterfaceProfile のシーケンス
  # - port_ref          [Port Object 型] Port 自身のオブジェクトリファレンス
  # - connector_profile [ConnectorProfileList 型] Port が現在保持する
  #                     ConnectorProfile のシーケンス
  # - owner             [RTObject Object 型] この Port を所有する
  #                     RTObjectのリファレンス
  # - properties        [NVList 型] その他のプロパティ。
  #
  # @param self
  #
  # @return PortProfile
  #
  # @else
  #
  # @brief [CORBA interface] Get the PortProfile of the Port
  #
  # This operation returns the PortProfile of the Port.
  # PortProfile struct has the following members,
  #
  # - name              [string ] The name of the Port.
  # - interfaces        [PortInterfaceProfileList] The sequence of 
  #                     PortInterfaceProfile owned by the Port
  # - port_ref          [Port Object] The object reference of the Port.
  # - connector_profile [ConnectorProfileList] The sequence of 
  #                     ConnectorProfile owned by the Port.
  # - owner             [RTObject Object] The object reference of 
  #                     RTObject that is owner of the Port.
  # - properties        [NVList] The other properties.
  #
  # @return the PortProfile of the Port
  #
  # @endif
  # PortProfile* get_port_profile()
  def get_port_profile(self):
    self._rtcout.RTC_TRACE("get_port_profile()")

    self.updateConnectors()

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)

    prof = RTC.PortProfile(self._profile.name,
                           self._profile.interfaces,
                           self._profile.port_ref,
                           self._profile.connector_profiles,
                           self._profile.owner,
                           self._profile.properties)

    return prof


  ##
  # @if jp
  #
  # @brief PortProfile を取得する。
  #
  # この関数は、オブジェクト内部に保持されている PortProfile の
  # const 参照を返す const 関数である。
  # 
  # @post この関数を呼び出すことにより内部状態が変更されることはない。
  #
  # @return PortProfile
  #
  # @else
  #
  # @brief Get the PortProfile of the Port
  #
  # This function is a const function that returns a const
  # reference of the PortProfile stored in this Port.
  #
  # @post This function never changes the state of the object.
  #
  # @return PortProfile
  #
  # @endif
  # PortProfile& getPortProfile() const;
  def getPortProfile(self):
    self._rtcout.RTC_TRACE("getPortProfile()")
    return self._profile


  ##
  # @if jp
  #
  # @brief [CORBA interface] ConnectorProfileListを取得する
  #
  # Portが保持する ConnectorProfile の sequence を返す。
  # ConnectorProfile は Port 間の接続プロファイル情報を保持する構造体であり、
  # 接続時にPort間で情報交換を行い、関連するすべての Port で同一の値が
  # 保持される。
  # ConnectorProfile は以下のメンバーを保持している。
  #
  # - name         [string 型] このコネクタの名前。
  # - connector_id [string 型] ユニークなID
  # - ports        [Port sequnce] このコネクタに関連する Port のオブジェクト
  #                リファレンスのシーケンス。
  # - properties   [NVList 型] その他のプロパティ。
  #
  # @param self
  #
  # @return この Port が保持する ConnectorProfile
  #
  # @else
  #
  # @brief [CORBA interface] Get the ConnectorProfileList of the Port
  #
  # This operation returns a list of the ConnectorProfiles of the Port.
  # ConnectorProfile includes the connection information that describes 
  # relation between (among) Ports, and Ports exchange the ConnectionProfile
  # on connection process and hold the same information in each Port.
  # ConnectionProfile has the following members,
  #
  # - name         [string] The name of the connection.
  # - connector_id [string] Unique identifier.
  # - ports        [Port sequnce] The sequence of Port's object reference
  #                that are related the connection.
  # - properties   [NVList] The other properties.
  #
  # @return the ConnectorProfileList of the Port
  #
  # @endif
  # virtual ConnectorProfileList* get_connector_profiles()
  def get_connector_profiles(self):
    self._rtcout.RTC_TRACE("get_connector_profiles()")

    self.updateConnectors()

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    return self._profile.connector_profiles


  ##
  # @if jp
  #
  # @brief [CORBA interface] ConnectorProfile を取得する
  #
  # connector_id で指定された ConnectorProfile を返す。
  # 指定した connector_id を持つ ConnectorProfile を保持していない場合は、
  # 空の ConnectorProfile を返す。
  #
  # @param self
  # @param connector_id ConnectorProfile の ID
  #
  # @return connector_id で指定された ConnectorProfile
  #
  # @else
  #
  # @brief [CORBA interface] Get the ConnectorProfile
  #
  # This operation returns the ConnectorProfiles specified connector_id.
  #
  # @param connector_id ID of the ConnectorProfile
  #
  # @return the ConnectorProfile identified by the connector_id
  #
  # @endif
  # ConnectorProfile* get_connector_profile(const char* connector_id)
  def get_connector_profile(self, connector_id):
    self._rtcout.RTC_TRACE("get_connector_profile(%s)", connector_id)

    self.updateConnectors()

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._profile.connector_profiles,
                                            self.find_conn_id(connector_id))
    if index < 0:
      conn_prof = RTC.ConnectorProfile("","",[],[])
      return conn_prof

    conn_prof = RTC.ConnectorProfile(self._profile.connector_profiles[index].name,
                                     self._profile.connector_profiles[index].connector_id,
                                     self._profile.connector_profiles[index].ports,
                                     self._profile.connector_profiles[index].properties)
    return conn_prof


  ##
  # @if jp
  #
  # @brief [CORBA interface] Port の接続を行う
  #
  # 与えられた ConnectoionProfile の情報に基づき、Port間の接続を確立
  # する。この関数は主にアプリケーションプログラムやツールから呼び出
  # すことを前提としている。
  # 
  # @pre アプリケーションプログラムは、コンポーネント間の複数の
  # Port を接続するために、適切な値をセットした ConnectorProfile を
  # connect() の引数として与えて呼び出さなければならない。
  #
  # @pre connect() に与える ConnectorProfile のメンバーのうち、
  # name, ports, properties メンバーに対してデータをセットしなければ
  # ならない。connector_id には通常空文字を設定するか、適当なUUIDを
  # 文字列で設定する必要がある。
  #
  # @pre ConnectorProfile::name は接続につける名前で CORBA::string
  # 型に格納できる任意の文字列である必要がある。
  # 
  # @pre ConnectorProfile::connector_id はすべての接続に対して一意な
  # ID (通常はUUID) が格納される。UUIDの設定は connect() 関数内で行
  # われるので、呼び出し側は空文字を設定する。既存の接続と同じUUIDを
  # 設定し connect() を呼び出した場合には PRECONDITION_NOT_MET エラー
  # を返す。ただし、将来の拡張で既存の接続プロファイルを更新するため
  # に既存の UUID を設定して呼び出す使用法が用いられる可能性がある。
  #
  # @pre ConnectorProfile::ports は RTC::PortService のシーケンスで、
  # 接続を構成する通常2つ以上のポートのオブジェクト参照を代入する必
  # 要がある。例外として、ポートのオブジェクト参照を1つだけ格納して
  # connect()を呼び出すことで、ポートのインターフェース情報を取得し
  # たり、特殊なポート(CORBAのRTC::PortService以外の相手)に対して接
  # 続を行う場合もある。
  #
  # @pre ConnectorProfile::properties はポートに関連付けられたインター
  # フェースに対するプロパティを与えるために使用する。プロパティは、
  # string 型をキー、Any 型を値としてもつペアのシーケンスであり、値
  # には任意のCORBAデータ型を格納できるが、可能な限り string 型とし
  # て格納されることが推奨される。
  #
  # @pre 以上 connect() 呼び出し時に設定する ConnectorProfile のメン
  # バをまとめると以下のようになる。
  #
  # - ConnectorProfile::name: 任意の接続名
  # - ConnectorProfile::connector_id: 空文字
  # - ConnectorProfile::ports: 1つ以上のポート
  # - ConnectorProfile::properties: インターフェースに対するプロパティ
  #
  # @post connect() 関数は、ConnectorProfile::portsに格納されたポー
  # トシーケンスの先頭のポートに対して notify_connect() を呼ぶ。
  #
  # @post notify_connect() は ConnectorProfile::ports に格納されたポー
  # ト順に notify_connect() をカスケード呼び出しする。このカスケード
  # 呼び出しは、途中のnotify_connect() でエラーが出てもポートのオブ
  # ジェクト参照が有効である限り、必ずすべてのポートに対して行われる
  # ことが保証される。有効でないオブジェクト参照がシーケンス中に存在
  # する場合、そのポートをスキップして、次のポートに対して
  # notify_connect() を呼び出す。
  #
  # @post connect() 関数は、notify_connect()の戻り値がRTC_OKであれば、
  # RTC_OK を返す。この時点で接続は完了する。RTC_OK以外
  # の場合は、この接続IDに対してdisconnect()を呼び出し接続を解除し、
  # notify_connect() が返したエラーリターンコードをそのまま返す。
  # 
  # @post connect() の引数として渡した ConnectorProfile には、
  # ConnectorProfile::connector_id および、途中のポートが
  # publishInterfaces() で公開したポートインターフェースの各種情報が
  # 格納されている。connect() および途中の notify_connect() が
  # ConnectorProfile::{name, ports} を変更することはない。
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
  # @pre To establish the connection among Ports of RT-Components,
  # application programs must call this operation giving
  # ConnectorProfile with valid values as an argument.
  #
  # @pre Out of ConnectorProfile member variables, "name", "ports"
  # and "properties" members shall be set valid
  # data. "connector_id" shall be set as empty string value or
  # valid string UUID value.
  #
  # @pre ConnectorProfile::name that is connection identifier shall
  # be any valid CORBA::string.
  # 
  #
  # @pre ConnectorProfile::connector_id shall be set unique
  # identifier (usually UUID is used) for all connections. Since
  # UUID string value is usually set in the connect() function,
  # caller should just set empty string. If the connect() is called
  # with the same UUID as existing connection, this function
  # returns PRECONDITION_NOT_MET error. However, in order to update
  # the existing connection profile, the "connect()" operation with
  # existing connector ID might be used as valid method by future
  # extension
  #
  # @pre ConnectorProfile::ports, which is sequence of
  # RTC::PortService references, shall store usually two or more
  # ports' references. As exceptions, the "connect()" operation
  # might be called with only one reference in ConnectorProfile, in
  # case of just getting interfaces information from the port, or
  # connecting a special port (i.e. the peer port except
  # RTC::PortService on CORBA).
  #
  # @pre ConnectorProfile::properties might be used to give certain
  # properties to the service interfaces associated with the port.
  # The properties is a sequence variable with a pair of key string
  # and Any type value. Although the A variable can store any type
  # of values, it is not recommended except string.
  #
  # @pre The following is the summary of the ConnectorProfile
  # member to be set when this operation is called.
  #
  # - ConnectorProfile::name: The any name of connection
  # - ConnectorProfile::connector_id: Empty string
  # - ConnectorProfile::ports: One or more port references
  # - ConnectorProfile::properties: Properties for the interfaces
  #
  # @post connect() operation will call the first port in the
  # sequence of the ConnectorProfile.
  #
  # @post "noify_connect()"s perform cascaded call to the ports
  # stored in the ConnectorProfile::ports by order. Even if errors
  # are raised by intermediate notify_connect() operation, as long
  # as ports' object references are valid, it is guaranteed that
  # this cascaded call is completed in all the ports.  If invalid
  # or dead ports exist in the port's sequence, the ports are
  # skipped and notify_connect() is called for the next valid port.
  #
  # @post connect() function returns RTC_OK if all the
  # notify_connect() return RTC_OK. At this time the connection is
  # completed.  If notify_connect()s return except RTC_OK,
  # connect() calls disconnect() operation with the connector_id to
  # destruct the connection, and then it returns error code from
  # notify_connect().
  #
  # @post The ConnectorProfile argument of the connect() operation
  # returns ConnectorProfile::connector_id and various information
  # about service interfaces that is published by
  # publishInterfaces() in the halfway ports. The connect() and
  # halfway notify_connect() functions never change
  # ConnectorProfile::{name, ports}.
  #
  # @param connector_profile The ConnectorProfile.
  # @return ReturnCode_t The return code of ReturnCode_t type.
  #
  # @endif
  #
  # virtual ReturnCode_t connect(ConnectorProfile& connector_profile)
  def connect(self, connector_profile):
    self._rtcout.RTC_TRACE("connect()")
    if self.isEmptyId(connector_profile):
      guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
      self.setUUID(connector_profile)
      assert(not self.isExistingConnId(connector_profile.connector_id))
      del guard
    else:
      guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
      if self.isExistingConnId(connector_profile.connector_id):
        self._rtcout.RTC_ERROR("Connection already exists.")
        return (RTC.PRECONDITION_NOT_MET,connector_profile)
      del guard

    try:
      retval,connector_profile = connector_profile.ports[0].notify_connect(connector_profile)
      if retval != RTC.RTC_OK:
        self._rtcout.RTC_ERROR("Connection failed. cleanup.")
        self.disconnect(connector_profile.connector_id)
    
      return (retval, connector_profile)
      #return connector_profile.ports[0].notify_connect(connector_profile)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      return (RTC.BAD_PARAMETER, connector_profile)

    return (RTC.RTC_ERROR, connector_profile)


  ##
  # @if jp
  #
  # @brief [CORBA interface] Port の接続通知を行う
  #
  # このオペレーションは、Port間の接続が行われる際に、Port間で内部的
  # に呼ばれるオペレーションであって、通常アプリケーションプログラム
  # や、Port以外のRTC関連オブジェクト等から呼び出されることは想定さ
  # れていない。
  #
  # notify_connect() 自体はテンプレートメソッドパターンとして、サブ
  # クラスで実装されることが前提の publishInterfaces(),
  # subscribeInterfaces() の2つの関数を内部で呼び出す。処理の手順は
  # 以下の通りである。
  #
  # - publishInterfaces(): インターフェース情報の公開
  # - connectNext(): 次の Port の notify_connect() の呼び出し
  # - subscribeInterfaces(): インターフェース情報の取得
  # - 接続情報の保存
  #
  # notify_connect() は ConnectorProfile::ports に格納されている
  # Port の順序に従って、カスケード状に呼び出しを行うことにより、イ
  # ンターフェース情報の公開と取得を関連すすべてのポートに対して行う。
  # このカスケード呼び出しは途中で中断されることはなく、必ず
  # ConnectorProfile::ports に格納されている全ポートに対して行われる。
  #
  # @pre notify_connect() は ConnectorProfile::ports 内に格納されて
  # いる Port 参照リストのうち、当該 Port 自身の参照の次に格納されて
  # いる Port に対して notify_connect() を呼び出す。したがって
  # ConnectorProfile::ports には当該 Port の参照が格納されている必要
  # がある。もし、自身の参照が格納されていない場合、その他の処理によ
  # りエラーが上書きされなければ、BAD_PARAMETER エラーが返される。
  #
  # @pre 呼び出し時に ConnectorProfile::connector_id には一意なIDと
  # して UUID が保持されている必要がある。通常 connector_id は
  # connect() 関数により与えられ、空文字の場合は動作は未定義である。
  #
  # @post ConnectorProfile::name, ConnectorProfile::connector_id,
  # ConnectorProfile::ports は notify_connect() の呼び出しにより
  # 書き換えられることはなく不変である。
  #
  # @post ConnectorProfile::properties は notify_connect() の内部で、
  # 当該 Port が持つサービスインターフェースに関する情報を他の Port
  # に伝えるために、プロパティ情報が書き込まれる。
  #
  # @post なお、ConnectorProfile::ports のリストの最初 Port の
  # notify_connet() が終了した時点では、すべての関連する Port の
  # notify_connect() の呼び出しが完了する。publishInterfaces(),
  # connectNext(), subscribeInterfaces() および接続情報の保存のいず
  # れかの段階でエラーが発生した場合でも、エラーコードは内部的に保持
  # されており、最初に生じたエラーのエラーコードが返される。
  #
  # @param connector_profile ConnectorProfile
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [CORBA interface] Notify the Ports connection
  #
  # This operation is usually called from other ports' connect() or
  # notify_connect() operations when connection between ports is
  # established.  This function is not premised on calling from
  # other functions or application programs.
  #
  # According to the template method pattern, the notify_connect()
  # calls "publishInterfaces()" and "subsctiveInterfaces()"
  # functions, which are premised on implementing in the
  # subclasses. The processing sequence is as follows.
  #
  # - publishInterfaces(): Publishing interface information
  # - connectNext(): Calling notify_connect() of the next port
  # - subscribeInterfaces(): Subscribing interface information
  # - Storing connection profile
  #
  # According to the order of port's references stored in the
  # ConnectorProfile::ports, publishing interface information to
  # all the ports and subscription interface information from all
  # the ports is performed by "notify_connect()"s.  This cascaded
  # call never aborts in the halfway operations, and calling
  # sequence shall be completed for all the ports.
  #
  # @pre notify_connect() calls notify_connect() for the port's
  # reference that is stored in next of this port's reference in
  # the sequence of the ConnectorProfile::ports. Therefore the
  # reference of this port shall be stored in the
  # ConnectorProfile::ports. If this port's reference is not stored
  # in the sequence, BAD_PARAMETER error will be returned, except
  # the return code is overwritten by other operations.
  #
  # @pre UUID shall be set to ConnectorProfile::connector_id as a
  # unique identifier when this operation is called.  Usually,
  # connector_id is given by a connect() function and, the behavior
  # is undefined in the case of a null character.
  #
  # @post ConnectorProfile::name, ConnectorProfile::connector_id,
  # ConnectorProfile::ports are invariant, and they are never
  # rewritten by notify_connect() operations.
  #
  # @post In order to transfer interface information to other
  # ports, interface property information is stored into the
  # ConnectorProfile::properties.
  #
  # @post At the end of notify_connect() operation for the first
  # port stored in the ConnectorProfile::ports sequence, the
  # related ports' notify_connect() invocations complete. Even if
  # errors are raised at the halfway of publishInterfaces(),
  # connectNext(), subscribeInterfaces() and storing process of
  # ConnectorProfile, error codes are saved and the first error is
  # returned.
  #
  # @param connector_profile The ConnectorProfile.
  # @return ReturnCode_t The return code of ReturnCode_t type.
  #
  # @endif
  #
  # virtual ReturnCode_t notify_connect(ConnectorProfile& connector_profile)
  def notify_connect(self, connector_profile):
    self._rtcout.RTC_TRACE("notify_connect()")

    guard_connection = OpenRTM_aist.ScopedLock(self._connection_mutex)

    # publish owned interface information to the ConnectorProfile
    retval = [RTC.RTC_OK for i in range(3)]

    self.onNotifyConnect(self.getName(),connector_profile)
    retval[0] = self.publishInterfaces(connector_profile)
    if retval[0] != RTC.RTC_OK:
      self._rtcout.RTC_ERROR("publishInterfaces() in notify_connect() failed.")

    self.onPublishInterfaces(self.getName(), connector_profile, retval[0])
    if self._onPublishInterfaces:
      self._onPublishInterfaces(connector_profile)

    # call notify_connect() of the next Port
    retval[1], connector_profile = self.connectNext(connector_profile)
    if retval[1] != RTC.RTC_OK:
      self._rtcout.RTC_ERROR("connectNext() in notify_connect() failed.")

    self.onConnectNextport(self.getName(), connector_profile, retval[1])
    # subscribe interface from the ConnectorProfile's information
    if self._onSubscribeInterfaces:
      self._onSubscribeInterfaces(connector_profile)

    retval[2] = self.subscribeInterfaces(connector_profile)
    if retval[2] != RTC.RTC_OK:
      self._rtcout.RTC_ERROR("subscribeInterfaces() in notify_connect() failed.")
      #self.notify_disconnect(connector_profile.connector_id)

    self.onSubscribeInterfaces(self.getName(), connector_profile, retval[2])

    self._rtcout.RTC_PARANOID("%d connectors are existing",
                              len(self._profile.connector_profiles))

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    # update ConnectorProfile
    index = self.findConnProfileIndex(connector_profile.connector_id)
    if index < 0:
      OpenRTM_aist.CORBA_SeqUtil.push_back(self._profile.connector_profiles,
                                           connector_profile)
      self._rtcout.RTC_PARANOID("New connector_id. Push backed.")

    else:
      self._profile.connector_profiles[index] = connector_profile
      self._rtcout.RTC_PARANOID("Existing connector_id. Updated.")

    for ret in retval:
      if ret != RTC.RTC_OK:
        self.onConnected(self.getName(), connector_profile, ret)
        return (ret, connector_profile)

    # connection established without errors
    if self._onConnected:
      self._onConnected(connector_profile)
    self.onConnected(self.getName(), connector_profile, RTC.RTC_OK)
    return (RTC.RTC_OK, connector_profile)


  ##
  # @if jp
  #
  # @brief [CORBA interface] Port の接続を解除する
  #
  # このオペレーションは与えられた connector_id に対応する接続を解除
  # する。connector_id は通常、システム全体において一意な UUID の文
  # 字列であり、事前に connect()/notify_connect() の呼び出しにより確
  # 立された接続プロファイル ConnectorProfile::connector_id に対応す
  # る。
  #
  # @pre connector_id は Port が保持する ConnectorProfile の少なくと
  # も一つの ID に一致する文字列でなければならない。当該 Port が持つ
  # ConnectorProfile のリスト内に connector_id と同一の ID を持つ
  # ConnectorProfile が存在しなければこの関数は BAD_PARAMETER エラー
  # を返す。
  #
  # @pre connector_id と同じ ID を持つ ConnectorProfile::ports には
  # 有効な Port の参照が含まれていなければならない。
  #
  # @post disconnect() 関数は、ConnectorProfile::ports の Port の参
  # 照リストの先頭に対して、notify_disconnect() を呼び出す。参照が無
  # 効であるなど、notify_disconnect() の呼び出しに失敗した場合には、
  # 参照リストの先頭から順番に成功するまで notify_disconnect() の呼
  # び出しを試す。notify_disconnect() の呼び出しに一つでも成功すれば、
  # notify_disconnect() の返却値をそのまま返し、一つも成功しなかった
  # 場合には RTC_ERROR エラーを返す。
  # 
  # @param connector_id ConnectorProfile の ID
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [CORBA interface] Disconnect the Port
  #
  # This operation destroys connection between this port and the
  # peer port according to given connector_id. Usually connector_id
  # should be a UUID string that is unique in the system.  And the
  # connection, which is established by connect()/notify_connect()
  # functions, is identified by the ConnectorProfile::connector_id.
  #
  # @pre connector_id shall be a character string which is same
  # with ID of at least one of the ConnectorProfiles stored in this
  # port. If ConnectorProfile that has same ID with the given
  # connector_id does not exist in the list of ConnectorProfile,
  # this operation returns BAD_PARAMTER error.
  #
  # @pre ConnectorProfile::ports that is same ID with given
  # connector_id shall store the valid ports' references.
  #
  # @post disconnect() function invokes the notify_disconnect() for
  # the port that is stored in the first of the
  # ConnectorProfile::ports. If notify_disconnect() call fails for
  # the first port, It tries on calling "notify_disconnect()" in
  # order for ports stored in ConnectorProfile::ports until the
  # operation call is succeeded. If notify_disconnect() succeeded
  # for at least one port, it returns return code from
  # notify_disconnect(). If none of notify_connect() call
  # succeeded, it returns RTC_ERROR error.
  #
  # @param connector_id The ID of the ConnectorProfile.
  # @return ReturnCode_t The return code of ReturnCode_t type.
  #
  # @endif
  #
  # virtual ReturnCode_t disconnect(const char* connector_id)
  def disconnect(self, connector_id):
    self._rtcout.RTC_TRACE("disconnect(%s)", connector_id)

    index = self.findConnProfileIndex(connector_id)

    if index < 0:
      self._rtcout.RTC_ERROR("Invalid connector id: %s", connector_id)
      return RTC.BAD_PARAMETER

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    if index < len(self._profile.connector_profiles):
      prof = self._profile.connector_profiles[index]
    del guard
    
    if len(prof.ports) < 1:
      self._rtcout.RTC_FATAL("ConnectorProfile has empty port list.")
      return RTC.PRECONDITION_NOT_MET

    for i in range(len(prof.ports)):
      p = prof.ports[i]
      try:
        return p.notify_disconnect(connector_id)
      except:
        self._rtcout.RTC_WARN(OpenRTM_aist.Logger.print_exception())
        continue

    self._rtcout.RTC_ERROR("notify_disconnect() for all ports failed.")
    return RTC.RTC_ERROR


  ##
  # @if jp
  #
  # @brief [CORBA interface] Port の接続解除通知を行う
  #
  # このオペレーションは、Port間の接続解除が行われる際に、Port間で内
  # 部的に呼ばれるオペレーションであり、通常アプリケーションプログラ
  # ムや、 Port 以外の RTC 関連オブジェクト等から呼び出されることは
  # 想定されていない。
  #
  # notify_disconnect() 自体はテンプレートメソッドパターンとして、サ
  # ブクラスで実装されることが前提の unsubscribeInterfaces() 関数を
  # 内部で呼び出す。処理の手順は以下の通りである。
  #
  # - ConnectorProfile の検索
  # - 次の Port の notify_disconnect() 呼び出し
  # - unsubscribeInterfaces()
  # - ConnectorProfile の削除
  #
  # notify_disconnect() は ConnectorProfile::ports に格納されている
  # Port の順序に従って、カスケード状に呼び出しを行うことにより、接
  # 続の解除をすべての Port に通知する。
  #
  # @pre Port は与えられた connector_id に対応する ConnectorProfile
  # を保持していなければならない。
  #
  # @post connector_id に対応する ConnectorProfile が見つからない場
  # 合はBAD_PARAMETER エラーを返す。
  #
  # @post カスケード呼び出しを行う際には ConnectorProfile::ports に
  # 保持されている Port の参照リストのうち、自身の参照の次の参照に対
  # して notify_disconnect() を呼び出すが、その呼び出しで例外が発生
  # した場合には、呼び出しをスキップしリストの次の参照に対して
  # notify_disconnect() を呼び出す。一つも呼び出しに成功しない場合、
  # RTC_ERROR エラーコードを返す。
  #
  # @post なお、ConnectorProfile::ports のリストの最初 Port の
  # notify_disconnet() が終了した時点では、すべての関連する Port の
  # notify_disconnect() の呼び出しが完了する。
  # 
  # @param connector_id ConnectorProfile の ID
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [CORBA interface] Notify the Ports disconnection
  #
  # This operation is invoked between Ports internally when the
  # connection is destroied. Generally it is not premised on
  # calling from application programs or RTC objects except Port
  # object.
  #
  # According to the template method pattern, the
  # notify_disconnect() calls unsubsctiveInterfaces() function,
  # which are premised on implementing in the subclasses. The
  # processing sequence is as follows.
  #
  # - Searching ConnectorProfile
  # - Calling notify_disconnect() for the next port
  # - Unsubscribing interfaces
  # - Deleting ConnectorProfile
  #
  # notify_disconnect() notifies disconnection to all the ports by
  # cascaded call to the stored ports in the
  # ConnectorProfile::ports in order.
  #
  # @pre The port shall store the ConnectorProfile having same id
  # with connector_id.
  #
  # @post If ConnectorProfile of same ID with connector_id does not
  # exist, it returns BAD_PARAMETER error.
  #
  # @post For the cascaded call, this operation calls
  # noify_disconnect() for the port that is stored in the next of
  # this port in the ConnectorProfile::ports.  If the operation
  # call raises exception for some failure, it tries to call
  # notify_disconnect() and skips until the operation succeeded.
  # If none of operation call succeeded, it returns RTC_ERROR.
  #
  # @post At the end of notify_disconnect() operation for the first
  # port stored in the ConnectorProfile::ports sequence, the
  # related ports' notify_disconnect() invocations complete.
  #
  # @param connector_id The ID of the ConnectorProfile.
  # @return ReturnCode_t The return code of ReturnCode_t type.
  #
  # @endif
  #
  # virtual ReturnCode_t notify_disconnect(const char* connector_id)
  def notify_disconnect(self, connector_id):
    self._rtcout.RTC_TRACE("notify_disconnect(%s)", connector_id)

    guard_connection = OpenRTM_aist.ScopedLock(self._connection_mutex)
    # The Port of which the reference is stored in the beginning of
    # connectorProfile's PortServiceList is master Port.
    # The master Port has the responsibility of disconnecting all Ports.
    # The slave Ports have only responsibility of deleting its own
    # ConnectorProfile.

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)

    index = self.findConnProfileIndex(connector_id)

    if index < 0:
      self._rtcout.RTC_ERROR("Invalid connector id: %s", connector_id)
      return RTC.BAD_PARAMETER

    prof = RTC.ConnectorProfile(self._profile.connector_profiles[index].name,
                                self._profile.connector_profiles[index].connector_id,
                                self._profile.connector_profiles[index].ports,
                                self._profile.connector_profiles[index].properties)
    self.onNotifyDisconnect(self.getName(), prof)

    retval = self.disconnectNext(prof)
    self.onDisconnectNextport(self.getName(), prof, retval)

    if self._onUnsubscribeInterfaces:
      self._onUnsubscribeInterfaces(prof)
    self.onUnsubscribeInterfaces(self.getName(), prof)
    self.unsubscribeInterfaces(prof)

    if self._onDisconnected:
      self._onDisconnected(prof)

    OpenRTM_aist.CORBA_SeqUtil.erase(self._profile.connector_profiles, index)
    
    self.onDisconnected(self.getName(), prof, retval)
    return retval


  ##
  # @if jp
  #
  # @brief [CORBA interface] Port の全接続を解除する
  #
  # このオペレーションはこの Port に関連した全ての接続を解除する。
  #
  # @param self
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [CORBA interface] Connect the Port
  #
  # This operation destroys all connection channels owned by the Port.
  #
  # @return ReturnCode_t The return code of this operation.
  #
  # @endif
  # virtual ReturnCode_t disconnect_all()
  def disconnect_all(self):
    self._rtcout.RTC_TRACE("disconnect_all()")
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    plist = copy.deepcopy(self._profile.connector_profiles)
    del guard
    
    retcode = RTC.RTC_OK
    len_ = len(plist)
    self._rtcout.RTC_DEBUG("disconnecting %d connections.", len_)

    # disconnect all connections
    # Call disconnect() for each ConnectorProfile.
    for i in range(len_):
      tmpret = self.disconnect(plist[i].connector_id)
      if tmpret != RTC.RTC_OK:
        retcode = tmpret

    return retcode


  #============================================================
  # Local operations
  #============================================================

  ##
  # @if jp
  # @brief Port の名前を設定する
  #
  # Port の名前を設定する。この名前は Port が保持する PortProfile.name
  # に反映される。
  #
  # @param self
  # @param name Port の名前
  #
  # @else
  # @brief Set the name of this Port
  #
  # This operation sets the name of this Port. The given Port's name is
  # applied to Port's PortProfile.name.
  #
  # @param name The name of this Port.
  #
  # @endif
  # void setName(const char* name);
  def setName(self, name):
    self._rtcout.RTC_TRACE("setName(%s)", name)
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    self._profile.name = name
    return

  ##
  # @if jp
  # @brief Port の名前を取得する
  # @else
  # @brief Get the name of this Port
  # @return The name of this Port.
  # @endif
  #
  # const char* PortBase::getName() const
  def getName(self):
    self._rtcout.RTC_TRACE("getName() = %s", self._profile.name)
    return self._profile.name


  ##
  # @if jp
  # @brief PortProfileを取得する
  #
  # Portが保持する PortProfile の const 参照を返す。
  #
  # @param self
  #
  # @return この Port の PortProfile
  #
  # @else
  # @brief Get the PortProfile of the Port
  #
  # This operation returns const reference of the PortProfile.
  #
  # @return the PortProfile of the Port
  #
  # @endif
  # const PortProfile& getProfile() const;
  def getProfile(self):
    self._rtcout.RTC_TRACE("getProfile()")
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    return self._profile


  ##
  # @if jp
  #
  # @brief Port のオブジェクト参照を設定する
  #
  # このオペレーションは Port の PortProfile にこの Port 自身の
  # オブジェクト参照を設定する。
  #
  # @param self
  # @param port_ref この Port のオブジェクト参照
  #
  # @else
  #
  # @brief Set the object reference of this Port
  #
  # This operation sets the object reference itself
  # to the Port's PortProfile.
  #
  # @param The object reference of this Port.
  #
  # @endif
  # void setPortRef(PortService_ptr port_ref);
  def setPortRef(self, port_ref):
    self._rtcout.RTC_TRACE("setPortRef()")
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    self._profile.port_ref = port_ref


  ##
  # @if jp
  #
  # @brief Port のオブジェクト参照を取得する
  #
  # このオペレーションは Port の PortProfile が保持している
  # この Port 自身のオブジェクト参照を取得する。
  #
  # @param self
  #
  # @return この Port のオブジェクト参照
  #
  # @else
  #
  # @brief Get the object reference of this Port
  #
  # This operation returns the object reference
  # that is stored in the Port's PortProfile.
  #
  # @return The object reference of this Port.
  #
  # @endif
  # PortService_ptr getPortRef();
  def getPortRef(self):
    self._rtcout.RTC_TRACE("getPortRef()")
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    return self._profile.port_ref


  ##
  # @if jp
  #
  # @brief Port の owner の RTObject を指定する
  #
  # このオペレーションは Port の PortProfile.owner を設定する。
  #
  # @param self
  # @param owner この Port を所有する RTObject の参照
  #
  # @else
  #
  # @brief Set the owner RTObject of the Port
  #
  # This operation sets the owner RTObject of this Port.
  #
  # @param owner The owner RTObject's reference of this Port
  #
  # @endif
  # void setOwner(RTObject_ptr owner);
  def setOwner(self, owner):
    prof = owner.get_component_profile()
    self._ownerInstanceName = prof.instance_name
    self._rtcout.RTC_TRACE("setOwner(%s)", self._ownerInstanceName)

    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    plist = self._profile.name.split(".")
    if not self._ownerInstanceName:
      self._rtcout.RTC_ERROR("Owner is not set.")
      self._rtcout.RTC_ERROR("addXXXPort() should be called in onInitialize().")
    portname = self._ownerInstanceName+"."+plist[-1]

    self._profile.owner = owner
    self._profile.name = portname


  #============================================================
  # callbacks
  #============================================================

  ##
  # @if jp
  #
  # @brief インターフェースを公開する際に呼ばれるコールバックをセットする
  #
  # このオペレーションは、このポートが接続時に、ポート自身が持つサー
  # ビスインターフェース情報を公開するタイミングで呼ばれるコールバッ
  # クファンクタをセットする。
  #
  # コールバックファンクタの所有権は、呼び出し側にあり、オブジェクト
  # が必要なくなった時に解体するのは呼び出し側の責任である。
  #
  # このコールバックファンクタは、PortBaseクラスの仮想関数である
  # publishInterfaces() が呼ばれたあとに、同じ引数 ConnectorProfile と
  # ともに呼び出される。このコールバックを利用して、
  # publishInterfaces() が公開した ConnectorProfile を変更することが可
  # 能であるが、接続関係の不整合を招かないよう、ConnectorProfile の
  # 変更には注意を要する。
  #
  # @param on_publish ConnectionCallback のサブクラスオブジェクトのポインタ
  #
  # @else
  #
  # @brief Setting callback called on publish interfaces
  #
  # This operation sets a functor that is called after publishing
  # interfaces process when connecting between ports.
  #
  # Since the ownership of the callback functor object is owned by
  # the caller, it has the responsibility of object destruction.
  # 
  # The callback functor is called after calling
  # publishInterfaces() that is virtual member function of the
  # PortBase class with an argument of ConnectorProfile type that
  # is same as the argument of publishInterfaces() function.
  # Although by using this functor, you can modify the ConnectorProfile
  # published by publishInterfaces() function, the modification
  # should be done carefully for fear of causing connection
  # inconsistency.
  #
  # @param on_publish a pointer to ConnectionCallback's subclasses
  #
  # @endif
  #
  # void setOnPublishInterfaces(ConnectionCallback* on_publish);
  def setOnPublishInterfaces(self, on_publish):
    self._onPublishInterfaces = on_publish
    return


  ##
  # @if jp
  #
  # @brief インターフェースを取得する際に呼ばれるコールバックをセットする
  #
  # このオペレーションは、このポートが接続時に、相手のポートが持つサー
  # ビスインターフェース情報を取得するタイミングで呼ばれるコールバッ
  # クファンクタをセットする。
  #
  # コールバックファンクタの所有権は、呼び出し側にあり、オブジェクト
  # が必要なくなった時に解体するのは呼び出し側の責任である。
  #
  # このコールバックファンクタは、PortBaseクラスの仮想関数である
  # subscribeInterfaces() が呼ばれる前に、同じ引数 ConnectorProfile と
  # ともに呼び出される。このコールバックを利用して、
  # subscribeInterfaces() に与える ConnectorProfile を変更することが可
  # 能であるが、接続関係の不整合を招かないよう、ConnectorProfile の
  # 変更には注意を要する。
  #
  # @param on_subscribe ConnectionCallback のサブクラスオブジェクトのポインタ
  #
  # @else
  #
  # @brief Setting callback called on publish interfaces
  #
  # This operation sets a functor that is called before subscribing
  # interfaces process when connecting between ports.
  #
  # Since the ownership of the callback functor object is owned by
  # the caller, it has the responsibility of object destruction.
  # 
  # The callback functor is called before calling
  # subscribeInterfaces() that is virtual member function of the
  # PortBase class with an argument of ConnectorProfile type that
  # is same as the argument of subscribeInterfaces() function.
  # Although by using this functor, you can modify ConnectorProfile
  # argument for subscribeInterfaces() function, the modification
  # should be done carefully for fear of causing connection
  # inconsistency.
  #
  # @param on_subscribe a pointer to ConnectionCallback's subclasses
  #
  # @endif
  #
  #void setOnSubscribeInterfaces(ConnectionCallback* on_subscribe);
  def setOnSubscribeInterfaces(self, on_subscribe):
    self._onSubscribeInterfaces = on_subscribe
    return


  ##
  # @if jp
  #
  # @brief 接続完了時に呼ばれるコールバックをセットする
  #
  # このオペレーションは、このポートが接続完了時に呼ばれる、コールバッ
  # クファンクタをセットする。
  #
  # コールバックファンクタの所有権は、呼び出し側にあり、オブジェクト
  # が必要なくなった時に解体するのは呼び出し側の責任である。
  #
  # このコールバックファンクタは、ポートの接続実行関数である
  # notify_connect() の終了直前に、接続処理が正常終了する際に限って
  # 呼び出されるコールバックである。接続処理の過程でエラーが発生した
  # 場合には呼び出されない。
  # 
  # このコールバックファンクタは notify_connect() が out パラメータ
  # として返すのと同じ引数 ConnectorProfile とともに呼び出されるので、
  # この接続において公開されたすべてのインターフェース情報を得ること
  # ができる。このコールバックを利用して、notify_connect() が返す
  # ConnectorProfile を変更することが可能であるが、接続関係の不整合
  # を招かないよう、ConnectorProfile の変更には注意を要する。
  #
  # @param on_subscribe ConnectionCallback のサブクラスオブジェクトのポインタ
  #
  # @else
  #
  # @brief Setting callback called on connection established
  #
  # This operation sets a functor that is called when connection
  # between ports established.
  #
  # Since the ownership of the callback functor object is owned by
  # the caller, it has the responsibility of object destruction.
  # 
  # The callback functor is called only when notify_connect()
  # function successfully returns. In case of error, the functor
  # will not be called.
  #
  # Since this functor is called with ConnectorProfile argument
  # that is same as out-parameter of notify_connect() function, you
  # can get all the information of published interfaces of related
  # ports in the connection.  Although by using this functor, you
  # can modify ConnectorProfile argument for out-paramter of
  # notify_connect(), the modification should be done carefully for
  # fear of causing connection inconsistency.
  #
  # @param on_subscribe a pointer to ConnectionCallback's subclasses
  #
  # @endif
  #
  # void setOnConnected(ConnectionCallback* on_connected);
  def setOnConnected(self, on_connected):
    self._onConnected = on_connected
    return


  ##
  # @if jp
  #
  # @brief インターフェースを解放する際に呼ばれるコールバックをセットする
  #
  # このオペレーションは、このポートが接続時に、相手のポートが持つサー
  # ビスインターフェース情報を解放するタイミングで呼ばれるコールバッ
  # クファンクタをセットする。
  #
  # コールバックファンクタの所有権は、呼び出し側にあり、オブジェクト
  # が必要なくなった時に解体するのは呼び出し側の責任である。
  #
  # このコールバックファンクタは、PortBaseクラスの仮想関数である
  # unsubscribeInterfaces() が呼ばれる前に、同じ引数 ConnectorProfile と
  # ともに呼び出される。このコールバックを利用して、
  # unsubscribeInterfaces() に与える ConnectorProfile を変更することが可
  # 能であるが、接続関係の不整合を招かないよう、ConnectorProfile の
  # 変更には注意を要する。
  #
  # @param on_unsubscribe ConnectionCallback のサブクラスオブジェク
  # トのポインタ
  #
  # @else
  #
  # @brief Setting callback called on unsubscribe interfaces
  #
  # This operation sets a functor that is called before unsubscribing
  # interfaces process when disconnecting between ports.
  #
  # Since the ownership of the callback functor object is owned by
  # the caller, it has the responsibility of object destruction.
  # 
  # The callback functor is called before calling
  # unsubscribeInterfaces() that is virtual member function of the
  # PortBase class with an argument of ConnectorProfile type that
  # is same as the argument of unsubscribeInterfaces() function.
  # Although by using this functor, you can modify ConnectorProfile
  # argument for unsubscribeInterfaces() function, the modification
  # should be done carefully for fear of causing connection
  # inconsistency.
  #
  # @param on_unsubscribe a pointer to ConnectionCallback's subclasses
  #
  # @endif
  #
  # void setOnUnsubscribeInterfaces(ConnectionCallback* on_subscribe);
  def setOnUnsubscribeInterfaces(self, on_subscribe):
    self._onUnsubscribeInterfaces = on_subscribe
    return


  ##
  # @if jp
  #
  # @brief 接続解除に呼ばれるコールバックをセットする
  #
  # このオペレーションは、このポートの接続解除時に呼ばれる、コールバッ
  # クファンクタをセットする。
  #
  # コールバックファンクタの所有権は、呼び出し側にあり、オブジェクト
  # が必要なくなった時に解体するのは呼び出し側の責任である。
  #
  # このコールバックファンクタは、ポートの接続解除実行関数である
  # notify_disconnect() の終了直前に、呼び出されるコールバックである。
  # 
  # このコールバックファンクタは接続に対応する ConnectorProfile とと
  # もに呼び出される。この ConnectorProfile はこのファンクタ呼出し後
  # に破棄されるので、変更がほかに影響を与えることはない。
  #
  # @param on_disconnected ConnectionCallback のサブクラスオブジェク
  # トのポインタ
  #
  # @else
  #
  # @brief Setting callback called on disconnected
  #
  # This operation sets a functor that is called when connection
  # between ports is destructed.
  #
  # Since the ownership of the callback functor object is owned by
  # the caller, it has the responsibility of object destruction.
  # 
  # The callback functor is called just before notify_disconnect()
  # that is disconnection execution function returns.
  #
  # This functor is called with argument of corresponding
  # ConnectorProfile.  Since this ConnectorProfile will be
  # destructed after calling this functor, modifications never
  # affect others.
  #
  # @param on_disconnected a pointer to ConnectionCallback's subclasses
  #
  # @endif
  #
  # void setOnDisconnected(ConnectionCallback* on_disconnected);
  def setOnDisconnected(self, on_disconnected):
    self._onDisconnected = on_disconnected
    return

  # void setOnConnectionLost(ConnectionCallback* on_connection_lost);
  def setOnConnectionLost(self, on_connection_lost):
    self._onConnectionLost = on_connection_lost
    return


  ##
  # @if jp
  # @brief PortConnectListeners のホルダをセットする
  #
  # ポートの接続に関するリスナ群を保持するホルダクラスへのポインタを
  # セットする。この関数は通常親のRTObjectから呼ばれ、RTObjectが持つ
  # ホルダクラスへのポインタがセットされる。
  #
  # @param portconnListeners PortConnectListeners オブジェクトのポインタ
  #
  # @else
  # @brief Setting PortConnectListener holder
  #
  # This operation sets a functor that is called when connection
  # of this port does lost. 
  #
  # @param on_connection_lost a pointer to ConnectionCallback's subclasses
  #
  # @endif
  #
  # void setPortConnectListenerHolder(PortConnectListeners* portconnListeners);
  def setPortConnectListenerHolder(self, portconnListeners):
    self._portconnListeners = portconnListeners
    return


  ##
  # @if jp
  #
  # @brief Interface 情報を公開する(サブクラス実装用)
  #
  # このオペレーションは、notify_connect() 処理シーケンスの始めにコール
  # される関数である。
  # notify_connect() では、
  #
  # - publishInterfaces()
  # - connectNext()
  # - subscribeInterfaces()
  # - updateConnectorProfile()
  #
  # の順に protected 関数がコールされ接続処理が行われる。
  # <br>
  # 具象 Port ではこのオペレーションをオーバーライドし、引数として
  # 与えられた ConnectorProfile に従い処理を行い、パラメータが不適切
  # であれば、RteurnCode_t 型のエラーコードを返す。
  # 通常 publishInterafaces() 内においては、この Port に属する
  # インターフェースに関する情報を ConnectorProfile に対して適切に設定し
  # 他の Port に通知しなければならない。
  # <br>
  # また、この関数がコールされる段階では、他の Port の Interface に関する
  # 情報はすべて含まれていないので、他の Port の Interface を取得する処理
  # は subscribeInterfaces() 内で行われるべきである。
  # <br>
  # このオペレーションは、新規の connector_id に対しては接続の生成、
  # 既存の connector_id に対しては更新が適切に行われる必要がある。<BR>
  # ※サブクラスでの実装参照用
  #
  # @param self
  # @param connector_profile 接続に関するプロファイル情報
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief Publish interface information
  #
  # This operation is pure virutal method that would be called at the
  # beginning of the notify_connect() process sequence.
  # In the notify_connect(), the following methods would be called in order.
  #
  # - publishInterfaces()
  # - connectNext()
  # - subscribeInterfaces()
  # - updateConnectorProfile() 
  #
  # In the concrete Port, this method should be overridden. This method
  # processes the given ConnectorProfile argument and if the given parameter
  # is invalid, it would return error code of ReturnCode_t.
  # Usually, publishInterfaces() method should set interfaces information
  # owned by this Port, and publish it to the other Ports.
  # <br>
  # When this method is called, other Ports' interfaces information may not
  # be completed. Therefore, the process to obtain other Port's interfaces
  # information should be done in the subscribeInterfaces() method.
  # <br>
  # This operation should create the new connection for the new
  # connector_id, and should update the connection for the existing
  # connection_id.
  #
  # @param connector_profile The connection profile information
  # @return The return code of ReturnCode_t type.
  #
  #@endif
  def publishInterfaces(self, connector_profile):
    pass


  ##
  # @if jp
  #
  # @brief 次の Port に対して notify_connect() をコールする
  #
  # ConnectorProfile の port_ref 内に格納されている Port のオブジェクト
  # リファレンスのシーケンスの中から、自身の Port の次の Port に対して
  # notify_connect() をコールする。
  #
  # @param self
  # @param connector_profile 接続に関するプロファイル情報
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief Call notify_connect() of the next Port
  #
  # This operation calls the notify_connect() of the next Port's 
  # that stored in ConnectorProfile's port_ref sequence.
  #
  # @param connector_profile The connection profile information
  #
  # @return The return code of ReturnCode_t type.
  #
  # @endif
  # virtual ReturnCode_t connectNext(ConnectorProfile& connector_profile);
  def connectNext(self, connector_profile):
    index = OpenRTM_aist.CORBA_SeqUtil.find(connector_profile.ports,
                                            self.find_port_ref(self._profile.port_ref))
    if index < 0:
      return (RTC.BAD_PARAMETER, connector_profile)

    index += 1
    if index < len(connector_profile.ports):
      p = connector_profile.ports[index]
      return p.notify_connect(connector_profile)

    return (RTC.RTC_OK, connector_profile)


  ##
  # @if jp
  #
  # @brief 次の Port に対して notify_disconnect() をコールする
  #
  # ConnectorProfile の port_ref 内に格納されている Port のオブジェクト
  # リファレンスのシーケンスの中から、自身の Port の次の Port に対して
  # notify_disconnect() をコールする。
  #
  # @param self
  # @param connector_profile 接続に関するプロファイル情報
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief Call notify_disconnect() of the next Port
  #
  # This operation calls the notify_disconnect() of the next Port's 
  # that stored in ConnectorProfile's port_ref sequence.
  #
  # @param connector_profile The connection profile information
  #
  # @return The return code of ReturnCode_t type.
  #
  # @endif
  # virtual ReturnCode_t disconnectNext(ConnectorProfile& connector_profile);
  def disconnectNext(self, connector_profile):
    index = OpenRTM_aist.CORBA_SeqUtil.find(connector_profile.ports,
                                            self.find_port_ref(self._profile.port_ref))
    if index < 0:
      return RTC.BAD_PARAMETER

    if index == (len(connector_profile.ports) - 1):
      return RTC.RTC_OK

    index += 1

    while index < len(connector_profile.ports):
      p = connector_profile.ports[index]
      index += 1
      try:
        return p.notify_disconnect(connector_profile.connector_id)
      except:
        self._rtcout.RTC_WARN(OpenRTM_aist.Logger.print_exception())
        continue

    return RTC.RTC_ERROR


  ##
  # @if jp
  #
  # @brief Interface 情報を取得する(サブクラス実装用)
  #
  # このオペレーションは、notify_connect() 処理シーケンスの中間にコール
  # される関数である。
  # notify_connect() では、
  #
  #  - publishInterfaces()
  #  - connectNext()
  #  - subscribeInterfaces()
  #  - updateConnectorProfile()
  #
  # の順に protected 関数がコールされ接続処理が行われる。
  # <br>
  # 具象 Port ではこのオペレーションをオーバーライドし、引数として
  # 与えられた ConnectorProfile に従い処理を行い、パラメータが不適切
  # であれば、RteurnCode_t 型のエラーコードを返す。
  # 引数 ConnectorProfile には他の Port の Interface に関する情報が
  # 全て含まれている。
  # 通常 subscribeInterafaces() 内においては、この Port が使用する
  # Interface に関する情報を取得し、要求側のインターフェースに対して
  # 情報を設定しなければならない。
  # <br>
  # このオペレーションは、新規の connector_id に対しては接続の生成、
  # 既存の connector_id に対しては更新が適切に行われる必要がある。<BR>
  # ※サブクラスでの実装参照用
  #
  # @param self
  # @param connector_profile 接続に関するプロファイル情報
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief Publish interface information
  #
  # This operation is pure virutal method that would be called at the
  # mid-flow of the notify_connect() process sequence.
  # In the notify_connect(), the following methods would be called in order.
  #
  #  - publishInterfaces()
  #  - connectNext()
  #  - subscribeInterfaces()
  #  - updateConnectorProfile()
  #
  # In the concrete Port, this method should be overridden. This method
  # processes the given ConnectorProfile argument and if the given parameter
  # is invalid, it would return error code of ReturnCode_t.
  # The given argument ConnectorProfile includes all the interfaces
  # information in it.
  # Usually, subscribeInterafaces() method obtains information of interfaces
  # from ConnectorProfile, and should set it to the interfaces that require
  # them.
  # <br>
  # This operation should create the new connection for the new
  # connector_id, and should update the connection for the existing
  # connection_id.
  #
  # @param connector_profile The connection profile information
  #
  # @return The return code of ReturnCode_t type.
  #
  #@endif
  def subscribeInterfaces(self, connector_profile):
    pass


  ##
  # @if jp
  #
  # @brief Interface の接続を解除する(サブクラス実装用)
  #
  # このオペレーションは、notify_disconnect() 処理シーケンスの終わりにコール
  # される関数である。
  # notify_disconnect() では、
  #  - disconnectNext()
  #  - unsubscribeInterfaces()
  #  - eraseConnectorProfile()
  # の順に protected 関数がコールされ接続解除処理が行われる。
  # <br>
  # 具象 Port ではこのオペレーションをオーバーライドし、引数として
  # 与えられた ConnectorProfile に従い接続解除処理を行う。<BR>
  # ※サブクラスでの実装参照用
  #
  # @param self
  # @param connector_profile 接続に関するプロファイル情報
  #
  # @else
  #
  # @brief Disconnect interface connection
  #
  # This operation is pure virutal method that would be called at the
  # end of the notify_disconnect() process sequence.
  # In the notify_disconnect(), the following methods would be called.
  #  - disconnectNext()
  #  - unsubscribeInterfaces()
  #  - eraseConnectorProfile() 
  # <br>
  # In the concrete Port, this method should be overridden. This method
  # processes the given ConnectorProfile argument and disconnect interface
  # connection.
  #
  # @param connector_profile The connection profile information
  #
  # @endif
  def unsubscribeInterfaces(self, connector_profile):
    pass


  ##
  # @if jp
  #
  # @brief 接続の最大数を設定する。
  #
  # @param limit_value 最大数
  #
  # @else
  #
  # @brief Set the maximum number of connections
  #
  #
  # @param limit_value The maximum number of connections
  #
  # @endif
  #
  # virtual void setConnectionLimit(int limit_value);
  def setConnectionLimit(self, limit_value):
    self._connectionLimit = limit_value
    return
    

  ##
  # @if jp
  # @brief Interface情報を公開する
  #
  # Interface情報を公開する。
  #
  #  dataport.dataflow_type
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  # @brief Publish interface information
  #
  # Publish interface information.
  #
  #
  # @return The return code of ReturnCode_t type
  #
  # @endif
  #
  # virtual ReturnCode_t _publishInterfaces(void);
  def _publishInterfaces(self):
    if not (self._connectionLimit < 0) :
      if self._connectionLimit <= len(self._profile.connector_profiles):
        self._rtcout.RTC_PARANOID("Connected number has reached the limitation.")
        self._rtcout.RTC_PARANOID("Can connect the port up to %d ports.",
                                  self._connectionLimit)
        self._rtcout.RTC_PARANOID("%d connectors are existing",
                                  len(self._profile.connector_profiles))
        return RTC.RTC_ERROR

    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief ConnectorProfile の connector_id フィールドが空かどうか判定
  #
  # 指定された ConnectorProfile の connector_id が空であるかどうかの判定を
  # 行う。
  #
  # @param self
  # @param connector_profile 判定対象コネクタプロファイル
  #
  # @return 引数で与えられた ConnectorProfile の connector_id が空であれば、
  #         true、そうでなければ false を返す。
  #
  # @else
  #
  # @brief Whether connector_id of ConnectorProfile is empty
  #
  # @return If the given ConnectorProfile's connector_id is empty string,
  #         it returns true.
  #
  # @endif
  # bool isEmptyId(const ConnectorProfile& connector_profile) const;
  def isEmptyId(self, connector_profile):
    return connector_profile.connector_id == ""


  ##
  # @if jp
  #
  # @brief UUIDを生成する
  #
  # このオペレーションは UUID を生成する。
  #
  # @param self
  #
  # @return uuid
  #
  # @else
  #
  # @brief Get the UUID
  #
  # This operation generates UUID.
  #
  # @return uuid
  #
  # @endif
  # const std::string getUUID() const;
  def getUUID(self):
    return str(OpenRTM_aist.uuid1())


  ##
  # @if jp
  #
  # @brief UUIDを生成し ConnectorProfile にセットする
  #
  # このオペレーションは UUID を生成し、ConnectorProfile にセットする。
  #
  # @param self
  # @param connector_profile connector_id をセットする ConnectorProfile
  #
  # @else
  #
  # @brief Create and set the UUID to the ConnectorProfile
  #
  # This operation generates and set UUID to the ConnectorProfile.
  #
  # @param connector_profile ConnectorProfile to be set connector_id
  #
  # @endif
  # void setUUID(ConnectorProfile& connector_profile) const;
  def setUUID(self, connector_profile):
    connector_profile.connector_id = self.getUUID()
    assert(connector_profile.connector_id != "")


  ##
  # @if jp
  #
  # @brief id が既存の ConnectorProfile のものかどうか判定する
  #
  # このオペレーションは与えられた ID が既存の ConnectorProfile のリスト中に
  # 存在するかどうか判定する。
  #
  # @param self
  # @param id_ 判定する connector_id
  #
  # @return id の存在判定結果
  #
  # @else
  #
  # @brief Whether the given id exists in stored ConnectorProfiles
  #
  # This operation returns boolean whether the given id exists in 
  # the Port's ConnectorProfiles.
  #
  # @param id connector_id to be find in Port's ConnectorProfiles
  #
  # @endif
  # bool isExistingConnId(const char* id);
  def isExistingConnId(self, id_):
    return OpenRTM_aist.CORBA_SeqUtil.find(self._profile.connector_profiles,
                                           self.find_conn_id(id_)) >= 0


  ##
  # @if jp
  #
  # @brief id を持つ ConnectorProfile を探す
  #
  # このオペレーションは与えられた ID を持つ ConnectorProfile を Port が
  # もつ ConnectorProfile のリスト中から探す。
  # もし、同一の id を持つ ConnectorProfile がなければ、空の ConnectorProfile
  # が返される。
  #
  # @param self
  # @param id_ 検索する connector_id
  #
  # @return connector_id を持つ ConnectorProfile
  #
  # @else
  #
  # @brief Find ConnectorProfile with id
  #
  # This operation returns ConnectorProfile with the given id from Port's
  # ConnectorProfiles' list.
  # If the ConnectorProfile with connector id that is identical with the
  # given id does not exist, empty ConnectorProfile is returned.
  #
  # @param id the connector_id to be searched in Port's ConnectorProfiles
  #
  # @return CoonectorProfile with connector_id
  #
  # @endif
  # ConnectorProfile findConnProfile(const char* id);
  def findConnProfile(self, id_):
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._profile.connector_profiles,
                                            self.find_conn_id(id_))
    if index < 0 or index >= len(self._profile.connector_profiles):
      return RTC.ConnectorProfile("","",[],[])

    return self._profile.connector_profiles[index]


  ##
  # @if jp
  #
  # @brief id を持つ ConnectorProfile を探す
  #
  # このオペレーションは与えられた ID を持つ ConnectorProfile を Port が
  # もつ ConnectorProfile のリスト中から探しインデックスを返す。
  # もし、同一の id を持つ ConnectorProfile がなければ、-1 を返す。
  #
  # @param self
  # @param id_ 検索する connector_id
  #
  # @return Port の ConnectorProfile リストのインデックス
  #
  # @else
  #
  # @brief Find ConnectorProfile with id
  #
  # This operation returns ConnectorProfile with the given id from Port's
  # ConnectorProfiles' list.
  # If the ConnectorProfile with connector id that is identical with the
  # given id does not exist, empty ConnectorProfile is returned.
  #
  # @param id the connector_id to be searched in Port's ConnectorProfiles
  #
  # @return The index of ConnectorProfile of the Port
  #
  # @endif
  # CORBA::Long findConnProfileIndex(const char* id);
  def findConnProfileIndex(self, id_):
    return OpenRTM_aist.CORBA_SeqUtil.find(self._profile.connector_profiles,
                                           self.find_conn_id(id_))


  ##
  # @if jp
  #
  # @brief ConnectorProfile の追加もしくは更新
  #
  # このオペレーションは与えられた与えられた ConnectorProfile を
  # Port に追加もしくは更新保存する。
  # 与えられた ConnectorProfile の connector_id と同じ ID を持つ
  # ConnectorProfile がリストになければ、リストに追加し、
  # 同じ ID が存在すれば ConnectorProfile を上書き保存する。
  #
  # @param self
  # @param connector_profile 追加もしくは更新する ConnectorProfile
  #
  # @else
  #
  # @brief Append or update the ConnectorProfile list
  #
  # This operation appends or updates ConnectorProfile of the Port
  # by the given ConnectorProfile.
  # If the connector_id of the given ConnectorProfile does not exist
  # in the Port's ConnectorProfile list, the given ConnectorProfile would be
  # append to the list. If the same id exists, the list would be updated.
  #
  # @param connector_profile the ConnectorProfile to be appended or updated
  #
  # @endif
  # void updateConnectorProfile(const ConnectorProfile& connector_profile);
  def updateConnectorProfile(self, connector_profile):
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._profile.connector_profiles,
                                            self.find_conn_id(connector_profile.connector_id))

    if index < 0:
      OpenRTM_aist.CORBA_SeqUtil.push_back(self._profile.connector_profiles,
                                           connector_profile)
    else:
      self._profile.connector_profiles[index] = connector_profile


  ##
  # @if jp
  #
  # @brief ConnectorProfile を削除する
  #
  # このオペレーションは Port の PortProfile が保持している
  # ConnectorProfileList のうち与えられた id を持つ ConnectorProfile
  # を削除する。
  #
  # @param self
  # @param id_ 削除する ConnectorProfile の id
  #
  # @return 正常に削除できた場合は true、
  #         指定した ConnectorProfile が見つからない場合は false を返す
  #
  # @else
  #
  # @brief Delete the ConnectorProfile
  #
  # This operation deletes a ConnectorProfile specified by id from
  # ConnectorProfileList owned by PortProfile of this Port.
  #
  # @param id The id of the ConnectorProfile to be deleted.
  #
  # @endif
  # bool eraseConnectorProfile(const char* id);
  def eraseConnectorProfile(self, id_):
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._profile.connector_profiles,
                                            self.find_conn_id(id_))

    if index < 0:
      return False

    OpenRTM_aist.CORBA_SeqUtil.erase(self._profile.connector_profiles, index)

    return True


  ##
  # @if jp
  #
  # @brief PortInterfaceProfile に インターフェースを登録する
  #
  # このオペレーションは Port が持つ PortProfile の、PortInterfaceProfile
  # にインターフェースの情報を追加する。
  # この情報は、get_port_profile() 似よって得られる PortProfile のうち
  # PortInterfaceProfile の値を変更するのみであり、実際にインターフェースを
  # 提供したり要求したりする場合には、サブクラスで、 publishInterface() ,
  #  subscribeInterface() 等の関数を適切にオーバーライドしインターフェースの
  # 提供、要求処理を行わなければならない。
  #
  # インターフェース(のインスタンス)名は Port 内で一意でなければならない。
  # 同名のインターフェースがすでに登録されている場合、この関数は false を
  # 返す。
  #
  # @param self
  # @param instance_name インターフェースのインスタンスの名前
  # @param type_name インターフェースの型の名前
  # @param pol インターフェースの属性 (RTC::PROVIDED もしくは RTC:REQUIRED)
  #
  # @return インターフェース登録処理結果。
  #         同名のインターフェースが既に登録されていれば false を返す。
  #
  # @else
  #
  # @brief Append an interface to the PortInterfaceProfile
  #
  # This operation appends interface information to the PortInterfaceProfile
  # that is owned by the Port.
  # The given interfaces information only updates PortInterfaceProfile of
  # PortProfile that is obtained through get_port_profile().
  # In order to provide and require interfaces, proper functions (for
  # example publishInterface(), subscribeInterface() and so on) should be
  # overridden in subclasses, and these functions provide concrete interface
  # connection and disconnection functionality.
  #
  # The interface (instance) name have to be unique in the Port.
  # If the given interface name is identical with stored interface name,
  # this function returns false.
  #
  # @param name The instance name of the interface.
  # @param type_name The type name of the interface.
  # @param pol The interface's polarity (RTC::PROVIDED or RTC:REQUIRED)
  #
  # @return false would be returned if the same name is already registered.
  #
  # @endif
  # bool appendInterface(const char* name, const char* type_name,
  #                      PortInterfacePolarity pol);
  def appendInterface(self, instance_name, type_name, pol):
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._profile.interfaces,
                                            self.find_interface(instance_name, pol))

    if index >= 0:
      return False

    # setup PortInterfaceProfile
    prof = RTC.PortInterfaceProfile(instance_name, type_name, pol)
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._profile.interfaces, prof)

    return True


  ##
  # @if jp
  #
  # @brief PortInterfaceProfile からインターフェース登録を削除する
  #
  # このオペレーションは Port が持つ PortProfile の、PortInterfaceProfile
  # からインターフェースの情報を削除する。
  #
  # @param self
  # @param name インターフェースのインスタンスの名前
  # @param pol インターフェースの属性 (RTC::PROVIDED もしくは RTC:REQUIRED)
  #
  # @return インターフェース削除処理結果。
  #         インターフェースが登録されていなければ false を返す。
  #
  # @else
  #
  # @brief Delete an interface from the PortInterfaceProfile
  #
  # This operation deletes interface information from the
  # PortInterfaceProfile that is owned by the Port.
  #
  # @param name The instance name of the interface.
  # @param pol The interface's polarity (RTC::PROVIDED or RTC:REQUIRED)
  #
  # @return false would be returned if the given name is not registered.
  #
  # @endif
  # bool deleteInterface(const char* name, PortInterfacePolarity pol);
  def deleteInterface(self, name, pol):
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._profile.interfaces,
                                            self.find_interface(name, pol))

    if index < 0:
      return False

    OpenRTM_aist.CORBA_SeqUtil.erase(self._profile.interfaces, index)
    return True


  ##
  # @if jp
  #
  # @brief PortProfile の properties に NameValue 値を追加する
  #
  # PortProfile の properties に NameValue 値を追加する。
  # 追加するデータの型をValueTypeで指定する。
  #
  # @param self
  # @param key properties の name
  # @param value properties の value
  #
  # @else
  #
  # @brief Add NameValue data to PortProfile's properties
  #
  # @param key The name of properties
  # @param value The value of properties
  #
  # @endif
  #  template <class ValueType>
  #  void addProperty(const char* key, ValueType value)
  def addProperty(self, key, value):
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._profile.properties,
                                         OpenRTM_aist.NVUtil.newNV(key, value))

  ##
  # @if jp
  #
  # @brief PortProfile の properties に NameValue 値を要素に追加する
  #
  # PortProfile の properties に NameValue 値を要素に追加する。
  #
  # @param key properties の name
  # @param value properties の value
  #
  # @else
  #
  # @brief Append NameValue data to PortProfile's properties
  #
  # Append NameValue data to PortProfile's properties.
  #
  # @param key The name of properties
  # @param value The value of properties
  #
  # @endif
  # void appendProperty(const char* key, const char* value)
  def appendProperty(self, key, value):
    OpenRTM_aist.NVUtil.appendStringValue(self._profile.properties, key, value)



  ##
  # @if jp
  #
  # @brief 存在しないポートをdisconnectする。
  #
  # @else
  #
  # @brief Disconnect ports that doesn't exist. 
  #
  # @endif
  # void updateConnectors()
  def updateConnectors(self):
    guard = OpenRTM_aist.ScopedLock(self._profile_mutex)
    
    connector_ids = []
    clist = self._profile.connector_profiles

    for cprof in clist:
      if not self.checkPorts(cprof.ports):
        connector_ids.append(cprof.connector_id)
        self._rtcout.RTC_WARN("Dead connection: %s", cprof.connector_id)

    for cid in connector_ids:
      self.disconnect(cid)

    return


  ##
  # @if jp
  #
  # @brief ポートの存在を確認する。
  #
  # @param ports 確認するポート
  # @return true:存在する,false:存在しない
  #
  # @else
  #
  # @brief Existence of ports
  #
  # @param ports Checked ports
  # @return true:existent,false:non existent
  #
  # @endif
  # bool checkPorts(::RTC::PortServiceList& ports)
  def checkPorts(self, ports):
    for port in ports:
      try:
        if port._non_existent():
          self._rtcout.RTC_WARN("Dead Port reference detected.")
          return False
      except:
        self._rtcout.RTC_WARN(OpenRTM_aist.Logger.print_exception())
        return False

    return True


  #inline void onNotifyConnect(const char* portname,
  #                            RTC::ConnectorProfile& profile)
  def onNotifyConnect(self, portname, profile):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectListenerType.ON_NOTIFY_CONNECT
      self._portconnListeners.portconnect_[type].notify(portname, profile)
    return


  #inline void onNotifyDisconnect(const char* portname,
  #                               RTC::ConnectorProfile& profile)
  def onNotifyDisconnect(self, portname, profile):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectListenerType.ON_NOTIFY_DISCONNECT
      self._portconnListeners.portconnect_[type].notify(portname, profile)
    return


  #inline void onUnsubscribeInterfaces(const char* portname,
  #                                    RTC::ConnectorProfile& profile)
  def onUnsubscribeInterfaces(self, portname, profile):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectListenerType.ON_UNSUBSCRIBE_INTERFACES
      self._portconnListeners.portconnect_[type].notify(portname, profile)
    return


  #inline void onPublishInterfaces(const char* portname,
  #                                RTC::ConnectorProfile& profile,
  #                                ReturnCode_t ret)
  def onPublishInterfaces(self, portname, profile, ret):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectRetListenerType.ON_PUBLISH_INTERFACES
      self._portconnListeners.portconnret_[type].notify(portname, profile, ret)
    return


  #inline void onConnectNextport(const char* portname,
  #                              RTC::ConnectorProfile& profile,
  #                              ReturnCode_t ret)
  def onConnectNextport(self, portname, profile, ret):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectRetListenerType.ON_CONNECT_NEXTPORT
      self._portconnListeners.portconnret_[type].notify(portname, profile, ret)
    return


  #inline void onSubscribeInterfaces(const char* portname,
  #                                  RTC::ConnectorProfile& profile,
  #                                  ReturnCode_t ret)
  def onSubscribeInterfaces(self, portname, profile, ret):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectRetListenerType.ON_SUBSCRIBE_INTERFACES
      self._portconnListeners.portconnret_[type].notify(portname, profile, ret)
    return


  #inline void onConnected(const char* portname,
  #                        RTC::ConnectorProfile& profile,
  #                        ReturnCode_t ret)
  def onConnected(self, portname, profile, ret):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectRetListenerType.ON_CONNECTED
      self._portconnListeners.portconnret_[type].notify(portname, profile, ret)
    return


  #inline void onDisconnectNextport(const char* portname,
  #                                 RTC::ConnectorProfile& profile,
  #                                 ReturnCode_t ret)
  def onDisconnectNextport(self, portname, profile, ret):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectRetListenerType.ON_DISCONNECT_NEXT
      self._portconnListeners.portconnret_[type].notify(portname, profile, ret)
    return


  #inline void onDisconnected(const char* portname,
  #                           RTC::ConnectorProfile& profile,
  #                           ReturnCode_t ret)
  def onDisconnected(self, portname, profile, ret):
    if self._portconnListeners != None:
      type = OpenRTM_aist.PortConnectRetListenerType.ON_DISCONNECTED
      self._portconnListeners.portconnret_[type].notify(portname, profile, ret)
    return



  #============================================================
  # Functor
  #============================================================

  ##
  # @if jp
  # @class if_name
  # @brief instance_name を持つ PortInterfaceProfile を探す Functor
  # @else
  # @brief A functor to find a PortInterfaceProfile named instance_name
  # @endif
  class if_name:
    def __init__(self, name):
      self._name = name

    def __call__(self, prof):
      return str(self._name) == str(prof.instance_name)
    

  ##
  # @if jp
  # @class find_conn_id
  # @brief id を持つ ConnectorProfile を探す Functor
  # @else
  # @brief A functor to find a ConnectorProfile named id
  # @endif
  class find_conn_id:
    def __init__(self, id_):
      """
       \param id_(string)
      """
      self._id = id_

    def __call__(self, cprof):
      """
       \param cprof(RTC.ConnectorProfile)
      """
      return str(self._id) == str(cprof.connector_id)

  ##
  # @if jp
  # @class find_port_ref
  # @brief コンストラクタ引数 port_ref と同じオブジェクト参照を探す Functor
  # @else
  # @brief A functor to find the object reference that is identical port_ref
  # @endif
  class find_port_ref:
    def __init__(self, port_ref):
      """
       \param port_ref(RTC.PortService)
      """
      self._port_ref = port_ref

    def __call__(self, port_ref):
      """
       \param port_ref(RTC.PortService)
      """
      return self._port_ref._is_equivalent(port_ref)

  ##
  # @if jp
  # @class connect_func
  # @brief Port の接続を行う Functor
  # @else
  # @brief A functor to connect Ports
  # @endif
  class connect_func:
    def __init__(self, p, prof):
      """
       \param p(RTC.PortService)
       \param prof(RTC.ConnectorProfile)
      """
      self._port_ref = p
      self._connector_profile = prof
      self.return_code = RTC.RTC_OK

    def __call__(self, p):
      """
       \param p(RTC.PortService)
      """
      if not self._port_ref._is_equivalent(p):
        retval = p.notify_connect(self._connector_profile)
        if retval != RTC.RTC_OK:
          self.return_code = retval

  ##
  # @if jp
  # @class disconnect_func
  # @brief Port の接続解除を行う Functor
  # @else
  # @brief A functor to disconnect Ports
  # @endif
  class disconnect_func:
    def __init__(self, p, prof):
      """
       \param p(RTC.PortService)
       \param prof(RTC.ConnectorProfile)
      """
      self._port_ref = p
      self._connector_profile = prof
      self.return_code = RTC.RTC_OK
      
    def __call__(self, p):
      """
       \param p(RTC.PortService)
      """
      if not self._port_ref._is_equivalent(p):
        retval = p.disconnect(self._connector_profile.connector_id)
        if retval != RTC.RTC_OK:
          self.return_code = retval

  ##
  # @if jp
  # @class disconnect_all_func
  # @brief Port の全接続解除を行う Functor
  # @else
  # @brief A functor to disconnect all Ports
  # @endif
  class disconnect_all_func:
    def __init__(self, p):
      """
       \param p(OpenRTM_aist.PortBase)
      """
      self.return_code = RTC.RTC_OK
      self._port = p

    def __call__(self, p):
      """
       \param p(RTC.ConnectorProfile)
      """
      retval = self._port.disconnect(p.connector_id)
      if retval != RTC.RTC_OK:
        self.return_code = retval

  ##
  # @if jp
  # @class find_interface
  # @brief name と polarity から interface を探す Functor
  # @else
  # @brief A functor to find interface from name and polarity
  # @endif
  class find_interface:
    def __init__(self, name, pol):
      """
       \param name(string)
       \param pol(RTC.PortInterfacePolarity)
      """
      self._name = name
      self._pol = pol

    def __call__(self, prof):
      """
       \param prof(RTC.PortInterfaceProfile)
      """
      name = prof.instance_name
      return (str(self._name) == str(name)) and (self._pol == prof.polarity)
