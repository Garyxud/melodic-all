#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file PortConnectListener.py
# @brief port's internal action listener classes
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2011
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import threading

class Lock:
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param mutex ロック用ミューテックス
  #
  # @else
  #
  # @endif
  def __init__(self, mutex):
    self.mutex = mutex
    self.mutex.acquire()


  ##
  # @if jp
  # @brief デストラクタ
  #
  # デストラクタ
  #
  # @param self
  #
  # @else
  #
  # @endif
  def __del__(self):
    self.mutex.release()

#============================================================

##
# @if jp
# @brief PortConnectListener のタイプ
#
# - ON_NOTIFY_CONNECT:         notify_connect() 関数内呼び出し直後
# - ON_NOTIFY_DISCONNECT:      notify_disconnect() 呼び出し直後
# - ON_UNSUBSCRIBE_INTERFACES: notify_disconnect() 内のIF購読解除時
#
# @else
# @brief The types of ConnectorDataListener
# 
# - ON_NOTIFY_CONNECT:         right after entering into notify_connect()
# - ON_NOTIFY_DISCONNECT:      right after entering into notify_disconnect()
# - ON_UNSUBSCRIBE_INTERFACES: unsubscribing IF in notify_disconnect()
#
# @endif
class PortConnectListenerType:
  """
  """

  ON_NOTIFY_CONNECT         = 0
  ON_NOTIFY_DISCONNECT      = 1
  ON_UNSUBSCRIBE_INTERFACES = 2
  PORT_CONNECT_LISTENER_NUM = 3

  def __init__(self):
    pass



##
# @if jp
# @class PortConnectListener クラス
# @brief PortConnectListener クラス
#
# 各アクションに対応するユーザーコードが呼ばれる直前のタイミング
# でコールされるリスナクラスの基底クラス。
#
# - ON_NOTIFY_CONNECT:         notify_connect() 関数内呼び出し直後
# - ON_NOTIFY_DISCONNECT:      notify_disconnect() 呼び出し直後
# - ON_UNSUBSCRIBE_INTERFACES: notify_disconnect() 内のIF購読解除時
#
# @else
# @class PortConnectListener class
# @brief PortConnectListener class
#
# This class is abstract base class for listener classes that
# provides callbacks for various events in rtobject.
#
# - ON_NOTIFY_CONNECT:         right after entering into notify_connect()
# - ON_NOTIFY_DISCONNECT:      right after entering into notify_disconnect()
# - ON_UNSUBSCRIBE_INTERFACES: unsubscribing IF in notify_disconnect()
#
# @endif
class PortConnectListener:
  """
  """

  def __init__(self):
    pass

  ##
  # @if jp
  #
  # @brief PortConnectListenerType を文字列に変換
  #
  # PortConnectListenerType を文字列に変換する
  #
  # @param type 変換対象 PortConnectListenerType
  #
  # @return 文字列変換結果
  #
  # @else
  #
  # @brief Convert PortConnectListenerType into the string.
  #
  # Convert PortConnectListenerType into the string.
  #
  # @param type The target PortConnectListenerType for transformation
  #
  # @return Trnasformation result of string representation
  #
  # @endif
  #static const char* toString(PortConnectListenerType type);
  def toString(type):
    typeString = ["ON_NOTIFY_CONNECT",
                  "ON_NOTIFY_DISCONNECT",
                  "ON_UNSUBSCRIBE_INTERFACES",
                  "ON_UPDATE_CONFIG_PARAM",
                  ""]
                      
    if type < ConfigurationParamListenerType.CONFIG_PARAM_LISTENER_NUM:
      return typeString[type]
        
    return "";

  toString = staticmethod(toString)


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
  #
  # @brief 仮想コールバック関数
  #
  # PortConnectListener のコールバック関数
  #
  # @else
  #
  # @brief Virtual Callback function
  #
  # This is a the Callback function for PortConnectListener.
  #
  # @endif
  #virtual void operator()(const char* portname,
  #                        RTC::ConnectorProfile& profile) = 0;
  def __call__(self, portname, profile):
    return



#============================================================
##
# @if jp
# @brief PortConnectRetListenerType のタイプ
#
# - ON_CONNECT_NEXTPORT:     notify_connect() 中のカスケード呼び出し直後
# - ON_SUBSCRIBE_INTERFACES: notify_connect() 中のインターフェース購読直後
# - ON_CONNECTED:            nofity_connect() 接続処理完了時に呼び出される
# - ON_DISCONNECT_NEXT:      notify_disconnect() 中にカスケード呼び出し直後
# - ON_DISCONNECTED:         notify_disconnect() リターン時
#
# @else
# @brief The types of PortConnectRetListenerType
# 
# - ON_CONNECT_NEXTPORT:     after cascade-call in notify_connect()
# - ON_SUBSCRIBE_INTERFACES: after IF subscribing in notify_connect()
# - ON_CONNECTED:            completed nofity_connect() connection process
# - ON_DISCONNECT_NEXT:      after cascade-call in notify_disconnect()
# - ON_DISCONNECTED:         completed notify_disconnect() disconnection
#
# @endif
class PortConnectRetListenerType:
  """
  """

  ON_PUBLISH_INTERFACES         = 0
  ON_CONNECT_NEXTPORT           = 1
  ON_SUBSCRIBE_INTERFACES       = 2
  ON_CONNECTED                  = 3
  ON_DISCONNECT_NEXT            = 4
  ON_DISCONNECTED               = 5
  PORT_CONNECT_RET_LISTENER_NUM = 6

  def __init__(self):
    pass



##
# @if jp
# @class PortConnectRetListener クラス
# @brief PortConnectRetListener クラス
#
# 各アクションに対応するユーザーコードが呼ばれる直前のタイミング
# でコールされるリスなクラスの基底クラス。
#
# - ON_PUBLISH_INTERFACES:   notify_connect() 中のインターフェース公開直後
# - ON_CONNECT_NEXTPORT:     notify_connect() 中のカスケード呼び出し直後
# - ON_SUBSCRIBE_INTERFACES: notify_connect() 中のインターフェース購読直後
# - ON_CONNECTED:            nofity_connect() 接続処理完了時に呼び出される
# - ON_DISCONNECT_NEXT:      notify_disconnect() 中にカスケード呼び出し直後
# - ON_DISCONNECTED:         notify_disconnect() リターン時
#
# @else
# @class PortConnectRetListener class
# @brief PortConnectRetListener class
#
# This class is abstract base class for listener classes that
# provides callbacks for various events in rtobject.
#
# - ON_CONNECT_NEXTPORT:     after cascade-call in notify_connect()
# - ON_SUBSCRIBE_INTERFACES: after IF subscribing in notify_connect()
# - ON_CONNECTED:            completed nofity_connect() connection process
# - ON_DISCONNECT_NEXT:      after cascade-call in notify_disconnect()
# - ON_DISCONNECTED:         completed notify_disconnect() disconnection
#
# @endif
class PortConnectRetListener:
  """
  """

  def __init__(self):
    pass


  ##
  # @if jp
  #
  # @brief PortConnectRetListenerType を文字列に変換
  #
  # PortConnectRetListenerType を文字列に変換する
  #
  # @param type 変換対象 PortConnectRetListenerType
  #
  # @return 文字列変換結果
  #
  # @else
  #
  # @brief Convert PortConnectRetListenerType into string.
  #
  # Convert PortConnectRetListenerType into string.
  #
  # @param type The target PortConnectRetListenerType for transformation
  #
  # @return Trnasformation result of string representation
  #
  # @endif
  #static const char* toString(PortConnectRetListenerType type);
  def toString(type):
    return
  toString = staticmethod(toString)


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
  #
  # @brief 仮想コールバック関数
  #
  # PortConnectRetListener のコールバック関数
  #
  # @else
  #
  # @brief Virtual Callback function
  #
  # This is a the Callback function for PortConnectRetListener.
  #
  # @endif
  #virtual void operator()(const char* portname,
  #                        RTC::ConnectorProfile& profile,
  #                        ReturnCode_t ret) = 0;
  def __call__(self, portname, profile, ret):
    pass



class Entry:
  def __init__(self,listener, autoclean):
    self.listener  = listener
    self.autoclean = autoclean
    return

#============================================================
##
# @if jp
# @class PortConnectListenerHolder 
# @brief PortConnectListener ホルダクラス
#
# 複数の PortConnectListener を保持し管理するクラス。
#
# @else
# @class PortConnectListenerHolder
# @brief PortConnectListener holder class
#
# This class manages one ore more instances of
# PortConnectListener class.
#
# @endif
class PortConnectListenerHolder:
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  # @else
  # @brief Constructor
  # @endif
  def __init__(self):
    self._listeners = []
    self._mutex = threading.RLock()
    return

    
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
  #
  # @brief リスナーの追加
  #
  # リスナーを追加する。
  #
  # @param listener 追加するリスナ
  # @param autoclean true:デストラクタで削除する,
  #                  false:デストラクタで削除しない
  # @else
  #
  # @brief Add the listener.
  #
  # This method adds the listener. 
  #
  # @param listener Added listener
  # @param autoclean true:The listener is deleted at the destructor.,
  #                  false:The listener is not deleted at the destructor. 
  # @endif
  #void addListener(PortConnectListener* listener, bool autoclean);
  def addListener(self, listener, autoclean):
    guard = Lock(self._mutex)
    self._listeners.append(Entry(listener, autoclean))
    del guard
    return

    
  ##
  # @if jp
  #
  # @brief リスナーの削除
  #
  # リスナを削除する。
  #
  # @param listener 削除するリスナ
  # @else
  #
  # @brief Remove the listener. 
  #
  # This method removes the listener. 
  #
  # @param listener Removed listener
  # @endif
  #void removeListener(PortConnectListener* listener);
  def removeListener(self, listener):
    guard = Lock(self._mutex)
    len_ = len(self._listeners)
    for i in range(len_):
      if (self._listeners[i].listener == listener) and self._listeners[i].autoclean:
        self._listeners[i].listener = None
      del self._listeners[i]
      del guard
      return
    del guard
    return


  ##
  # @if jp
  #
  # @brief リスナーへ通知する
  #
  # 登録されているリスナのコールバックメソッドを呼び出す。
  #
  # @param info ConnectorInfo
  # @else
  #
  # @brief Notify listeners. 
  #
  # This calls the Callback method of the registered listener. 
  #
  # @param info ConnectorInfo
  # @endif
  #void notify(const char* portname, RTC::ConnectorProfile& profile);
  def notify(self, portname, profile):
    guard = Lock(self._mutex)
    for listener in self._listeners:
      listener.listener(portname, profile)
    del guard
    return


##
# @if jp
# @class PortConnectRetListenerHolder
# @brief PortConnectRetListener ホルダクラス
#
# 複数の PortConnectRetListener を保持し管理するクラス。
#
# @else
# @class PortConnectRetListenerHolder
# @brief PortConnectRetListener holder class
#
# This class manages one ore more instances of
# PortConnectRetListener class.
#
# @endif
class PortConnectRetListenerHolder:
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  # @else
  # @brief Constructor
  # @endif
  #PortConnectRetListenerHolder();
  def __init__(self):
    self._listeners = []
    self._mutex = threading.RLock()
    return


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
  #
  # @brief リスナーの追加
  #
  # リスナーを追加する。
  #
  # @param listener 追加するリスナ
  # @param autoclean true:デストラクタで削除する,
  #                  false:デストラクタで削除しない
  # @else
  #
  # @brief Add the listener.
  #
  # This method adds the listener. 
  #
  # @param listener Added listener
  # @param autoclean true:The listener is deleted at the destructor.,
  #                  false:The listener is not deleted at the destructor. 
  # @endif
  #void addListener(PortConnectRetListener* listener, bool autoclean);
  def addListener(self, listener, autoclean):
    guard = Lock(self._mutex)
    self._listeners.append(Entry(listener, autoclean))
    del guard
    return

    
  ##
  # @if jp
  #
  # @brief リスナーの削除
  #
  # リスナを削除する。
  #
  # @param listener 削除するリスナ
  # @else
  #
  # @brief Remove the listener. 
  #
  # This method removes the listener. 
  #
  # @param listener Removed listener
  # @endif
  #void removeListener(PortConnectRetListener* listener);
  def removeListener(self, listener):
    guard = Lock(self._mutex)
    len_ = len(self._listeners)
    for i in range(len_):
      if (self._listeners[i].listener == listener) and self._listeners[i].autoclean:
        self._listeners[i].listener = None
      del self._listeners[i]
      del guard
      return
    del guard
    return

    
  ##
  # @if jp
  #
  # @brief リスナーへ通知する
  #
  # 登録されているリスナのコールバックメソッドを呼び出す。
  #
  # @param info ConnectorInfo
  # @param cdrdata データ
  # @else
  #
  # @brief Notify listeners. 
  #
  # This calls the Callback method of the registered listener. 
  #
  # @param info ConnectorInfo
  # @param cdrdata Data
  # @endif
  #void notify(const char* portname, RTC::ConnectorProfile& profile,
  #            ReturnCode_t ret);
  def notify(self, portname, profile, ret):
    guard = Lock(self._mutex)
    for listener in self._listeners:
      listener.listener(portname, profile, ret)
    del guard
    return



##
# @if jp
# @class PortConnectListeners
# @brief PortConnectListeners クラス
#
#
# @else
# @class PortConnectListeners
# @brief PortConnectListeners class
#
#
# @endif
class PortConnectListeners:
  """
  """

  def __init__(self):
    pass


  ##
  # @if jp
  # @brief PortConnectListenerType リスナ配列
  # PortConnectListenerType リスナを格納
  # @else
  # @brief PortConnectListenerType listener array
  # The PortConnectListenerType listener is stored. 
  # @endif
  portconnect_num = PortConnectListenerType.PORT_CONNECT_LISTENER_NUM
  portconnect_ = [PortConnectListenerHolder() for i in range(portconnect_num)]
    
  ##
  # @if jp
  # @brief PortConnectRetTypeリスナ配列
  # PortConnectRetTypeリスナを格納
  # @else
  # @brief PortConnectRetType listener array
  # The PortConnectRetType listener is stored.
  # @endif
  portconnret_num = PortConnectRetListenerType.PORT_CONNECT_RET_LISTENER_NUM
  portconnret_ = [PortConnectRetListenerHolder() for i in range(portconnret_num)]
