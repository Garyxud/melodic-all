#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# @file OutPort.py
# @brief OutPort class
# @date $Date: 2007/09/19$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2006-2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


from omniORB import *
from omniORB import any

import OpenRTM_aist


##
# @if jp
# @brief データにタイムスタンプをセットする
#
# データポートのデータに対してタイムスタンプをセットする。データポート
# のデータは構造体のメンバーとして tm.sec, tm.nsec を持つ必要がある。
#
# @param data タイムスタンプをセットするデータ。実行後実行時のタイムス
#             タンプがセットされる
#
# @else
# @brief Setting timestamp to data
#
# This function sets timestamp to data of data port. This data should
# have tm.sec, tm.nsec as members of the structure.
#
# @param data Data to be set timestamp. After executing this
#             function, current timestamp is set to the data.
#
# @endif
# template <class DataType>
# void setTimestamp(DataType& data)
def setTimestamp(data):
  # set timestamp
  tm = OpenRTM_aist.Time()
  data.tm.sec  = tm.sec
  data.tm.nsec = tm.usec * 1000


##
# @if jp
#
# @class OutPort
#
# @brief OutPort クラス
# 
# OutPort 用クラス
#
# @since 0.2.0
#
# @else
# 
# @endif
class OutPort(OpenRTM_aist.OutPortBase):
  """
  """



  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param name ポート名
  # @param value このポートにバインドされるデータ変数
  # @param buffer_ バッファ
  #
  # @else
  #
  # @brief Constructor
  #
  # @endif
  def __init__(self, name, value, buffer=None):
    OpenRTM_aist.OutPortBase.__init__(self, name, OpenRTM_aist.toTypename(value))
    self._value          = value
    #self._timeoutTick    = 1000 # timeout tick: 1ms
    #self._writeBlock     = False
    #self._writeTimeout   = 0
    self._OnWrite        = None
    self._OnWriteConvert = None
    #self._OnOverflow     = None
    #self._OnUnderflow    = None
    #self._OnConnect      = None
    #self._OnDisconnect   = None
    

  def __del__(self, OutPortBase=OpenRTM_aist.OutPortBase):
    OutPortBase.__del__(self)
    return

  ##
  # @if jp
  #
  # @brief データ書き込み
  #
  # ポートへデータを書き込む。
  #
  # - コールバックファンクタ OnWrite がセットされている場合、
  #   OutPort が保持するバッファに書き込む前に OnWrite が呼ばれる。
  # - OutPort が保持するバッファがオーバーフローを検出できるバッファであり、
  #   かつ、書き込む際にバッファがオーバーフローを検出した場合、
  #   コールバックファンクタ OnOverflow が呼ばれる。
  # - コールバックファンクタ OnWriteConvert がセットされている場合、
  #   バッファ書き込み時に、 OnWriteConvert の operator() の戻り値が
  #   バッファに書き込まれる。
  #
  # @param self
  # @param value 書き込み対象データ
  #
  # @return 書き込み処理結果(書き込み成功:true、書き込み失敗:false)
  #
  # @else
  #
  # @brief Write data
  #
  # @endif
  # virtual bool write(const DataType& value)
  ##
  # @if jp
  #
  # @brief データ書き込み
  #
  # ポートへデータを書き込む。
  # 設定された値をポートに書き込む。
  #
  # @param self
  # @param value 書き込み対象データ
  #
  # @return 書き込み処理結果(書き込み成功:true、書き込み失敗:false)
  #
  # @else
  #
  # @endif
  # bool operator<<(DataType& value)
  def write(self, value=None):
    if not value:
      value=self._value

    
    if self._OnWrite:
      self._OnWrite(value)

    # check number of connectors
    conn_size = len(self._connectors)
    if not conn_size > 0:
      return True
  
    # set timestamp
    #tm = Time()
    #value.tm.sec  = tm.sec
    #value.tm.nsec = tm.usec * 1000

    #tm_pre = Time()

    if self._OnWriteConvert:
      value = self._OnWriteConvert(value)
      
    result = True

    guard = OpenRTM_aist.ScopedLock(self._connector_mutex)
    for con in self._connectors:
      ret = con.write(value)
      if ret != self.PORT_OK:
        result = False
        if ret == self.CONNECTION_LOST:
          self.disconnect(con.id())

    return result


  ##
  # @if jp
  #
  # @brief データ書き込み処理のブロックモードの設定
  #
  # 書き込み処理に対してブロックモードを設定する。
  # ブロックモードを指定した場合、バッファに書き込む領域ができるか
  # タイムアウトが発生するまで write() メソッドの呼びだしがブロックされる。
  #
  # @param self
  # @param block ブロックモードフラグ
  #
  # @else
  #
  # @brief Set read() block mode
  #
  # @endif
  #def setWriteBlock(self, block):
  #  self._writeBlock = block


  ##
  # @if jp
  #
  # @brief 書き込み処理のタイムアウト時間の設定
  # 
  # write() のタイムアウト時間を usec で設定する。
  # write() はブロックモードでなければならない。
  #
  # @param self
  # @param timeout タイムアウト時間 [usec]
  #
  # @else
  #
  # @brief Set write() timeout
  #
  # @endif
  #def setWriteTimeout(self, timeout):
  #  self._writeTimeout = timeout


  ##
  # @if jp
  #
  # @brief OnWrite コールバックの設定
  #
  # データ書き込み直前に呼ばれる OnWrite コールバックファンクタを設定する。
  #
  # @param self
  # @param on_write OnWrite コールバックファンクタ
  #
  # @else
  #
  # @brief Set OnWrite callback
  #
  # @endif
  def setOnWrite(self, on_write):
    self._OnWrite = on_write


  ##
  # @if jp
  #
  # @brief OnWriteConvert コールバックの設定
  #
  # データ書き込み時に呼ばれる OnWriteConvert コールバックファンクタを設定
  # する。
  # このコールバック関数の処理結果が書き込まれる。
  # このため書き込みデータのフィルタリングが可能となる。
  #
  # @param self
  # @param on_wconvert OnWriteConvert コールバックファンクタ
  #
  # @else
  #
  # @brief Set OnWriteConvert callback
  #
  # @endif
  def setOnWriteConvert(self, on_wconvert):
    self._OnWriteConvert = on_wconvert


  ##
  # @if jp
  #
  # @brief データ型名取得用メソッド
  #
  # データの型名を取得するため、InPortCorbaProviderから呼ばれる。
  # 
  # @param self
  #
  # @return バッファに設定されているデータの型名
  #
  # @else
  #
  # @endif
  def getPortDataType(self):
    val = any.to_any(self._value)
    return str(val.typecode().name())



  class subscribe:
    def __init__(self, prof, subs = None):
      if subs:
        self._prof = subs._prof
        self._consumer = subs._consumer
        return

      self._prof = prof
      self._consumer = None
      

    def __call__(self, cons):
      if cons.subscribeInterface(self._prof.properties):
        self._consumer = cons
