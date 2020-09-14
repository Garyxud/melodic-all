#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file InPort.py
# @brief InPort template class
# @date $Date: 2007/09/20 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

from omniORB import *
from omniORB import any
import sys
import copy
import time

import OpenRTM_aist

##
# @if jp
#
# @class InPort
#
# @brief InPort クラス
# 
# InPort の実装クラス。
# InPort は内部にリングバッファを持ち、外部から送信されたデータを順次
# このリングバッファに格納する。リングバッファのサイズはデフォルトで64と
# なっているが、コンストラクタ引数によりサイズを指定することができる。
# データはフラグによって未読、既読状態が管理され、isNew(), getNewDataLen()
# getNewList(), getNewListReverse() 等のメソッドによりハンドリングすることが
# できる。
#
# @since 0.2.0
#
# @else
#
# @class InPort
#
# @brief InPort template class
#
# This class template provides interfaces to input port.
# Component developer can define input value, which act as input
# port from other components, using this template.
# This is class template. This class have to be incarnated class as port
# value types. This value types are previously define RtComponent IDL.
# ex. type T: TimedFload, TimedLong etc... 
#
# @since 0.2.0
#
# @endif
class InPort(OpenRTM_aist.InPortBase):
  """
  """



  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ。
  #
  # @param self
  # @param name InPort 名。InPortBase:name() により参照される。
  # @param value この InPort にバインドされる変数
  # @param read_block 読込ブロックフラグ。
  #        データ読込時に未読データがない場合、次のデータ受信までブロックする
  #        かどうかを設定(デフォルト値:False)
  # @param write_block 書込ブロックフラグ。
  #        データ書込時にバッファがフルであった場合、バッファに空きができる
  #        までブロックするかどうかを設定(デフォルト値:False)
  # @param read_timeout 読込ブロックを指定していない場合の、データ読取タイム
  #        アウト時間(ミリ秒)(デフォルト値:0)
  # @param write_timeout 書込ブロックを指定していない場合の、データ書込タイム
  #        アウト時間(ミリ秒)(デフォルト値:0)
  #
  # @else
  #
  # @brief A constructor.
  #
  # Setting channel name and registering channel value.
  #
  # @param self
  # @param name A name of the InPort. This name is referred by
  #             InPortBase::name().
  # @param value A channel value related with the channel.
  # @param read_block
  # @param write_block
  # @param read_timeout
  # @param write_timeout
  #
  # @endif
  def __init__(self, name, value, buffer=None,
               read_block=False, write_block=False,
               read_timeout=0, write_timeout = 0):
    OpenRTM_aist.InPortBase.__init__(self, name, OpenRTM_aist.toTypename(value))
    self._name           = name
    self._value          = value
    self._OnRead         = None
    self._OnReadConvert  = None


  def __del__(self, InPortBase=OpenRTM_aist.InPortBase):
    InPortBase.__del__(self)
    return

  ##
  # @if jp
  # @brief ポート名称を取得する。
  #
  # ポート名称を取得する。
  #
  # @param self
  #
  # @return ポート名称
  #
  # @else
  #
  # @endif
  #
  # const char* name()
  def name(self):
    return self._name


  ##
  # @if jp
  # @brief 最新データか確認
  #
  # 現在のバッファ位置に格納されているデータが最新データか確認する。
  #
  # @param self
  #
  # @return 最新データ確認結果
  #            ( true:最新データ．データはまだ読み出されていない
  #             false:過去のデータ．データは既に読み出されている)
  #
  # @else
  #
  # @endif
  #
  # bool isNew()
  def isNew(self):
    self._rtcout.RTC_TRACE("isNew()")

    if len(self._connectors) == 0:
      self._rtcout.RTC_DEBUG("no connectors")
      return False

    r = self._connectors[0].getBuffer().readable()
    if r > 0:
      self._rtcout.RTC_DEBUG("isNew() = True, readable data: %d",r)
      return True

    self._rtcout.RTC_DEBUG("isNew() = False, no readable data")
    return False


  ##
  # @if jp
  #
  # @brief バッファが空かどうか確認する
  # 
  # InPortのバッファが空かどうかを bool 値で返す。
  # 空の場合は true, 未読データがある場合は false を返す。
  #
  # @return true  バッファは空
  #         false バッファに未読データがある
  # 
  # @else
  #
  # @brief Check whether the data is newest
  # 
  # Check whether the data stored at a current buffer position is newest.
  #
  # @return Newest data check result
  #         ( true:Newest data. Data has not been readout yet.
  #          false:Past data．Data has already been readout.)
  # 
  # @endif
  #
  # bool isEmpty()
  def isEmpty(self):
    self._rtcout.RTC_TRACE("isEmpty()")

    if len(self._connectors) == 0:
      self._rtcout.RTC_DEBUG("no connectors")
      return True

    r = self._connectors[0].getBuffer().readable()
    if r == 0:
      self._rtcout.RTC_DEBUG("isEmpty() = true, buffer is empty")
      return True
      
    self._rtcout.RTC_DEBUG("isEmpty() = false, data exists in the buffer")
    return False


  ##
  # @if jp
  #
  # @brief DataPort から値を読み出す
  #
  # InPortに書き込まれたデータを読みだす。接続数が0、またはバッファに
  # データが書き込まれていない状態で読みだした場合の戻り値は不定である。
  # バッファが空の状態のとき、
  # 事前に設定されたモード (readback, do_nothing, block) に応じて、
  # 以下のような動作をする。
  #
  # - readback: 最後の値を読みなおす。
  #
  # - do_nothing: 何もしない
  #
  # - block: ブロックする。タイムアウトが設定されている場合は、
  #       タイムアウトするまで待つ。
  #
  # バッファが空の状態では、InPortにバインドされた変数の値が返される。
  # したがって、初回読み出し時には不定値を返す可能性がある。
  # この関数を利用する際には、
  #
  # - isNew(), isEmpty() と併用し、事前にバッファ状態をチェックする。
  # 
  # - 初回読み出し時に不定値を返さないようにバインド変数を事前に初期化する
  # 
  #
  # 各コールバック関数は以下のように呼び出される。
  # - OnRead: read() 関数が呼ばれる際に必ず呼ばれる。
  # 
  # - OnReadConvert: データの読み出しが成功した場合、読みだしたデータを
  #       引数としてOnReadConvertが呼び出され、戻り値をread()が戻り値
  #       として返す。
  #
  # - OnEmpty: バッファが空のためデータの読み出しに失敗した場合呼び出される。
  #        OnEmpty の戻り値を read() の戻り値として返す。
  #
  # - OnBufferTimeout: データフロー型がPush型の場合に、読み出し
  #        タイムアウトのためにデータの読み出しに失敗した場合に呼ばれる。
  #
  # - OnRecvTimeout: データフロー型がPull型の場合に、読み出しタイムアウト
  #        のためにデータ読み出しに失敗した場合に呼ばれる。
  #
  # - OnReadError: 上記以外の理由で読みだしに失敗した場合に呼ばれる。
  #        理由としては、バッファ設定の不整合、例外の発生などが考えられる
  #        が通常は起こりえないためバグの可能性がある。
  #
  # @return 読み出したデータ
  #
  # @else
  #
  # @brief Readout the value from DataPort
  #
  # Readout the value from DataPort
  #
  # - When Callback functor OnRead is already set, OnRead will be invoked
  #   before reading from the buffer held by DataPort.
  # - When the buffer held by DataPort can detect the underflow,
  #   and when it detected the underflow at reading, callback functor
  #   OnUnderflow will be invoked.
  # - When callback functor OnReadConvert is already set, the return value of
  #   operator() of OnReadConvert will be the return value of read().
  # - When timeout of reading is already set by setReadTimeout(),
  #   it waits for only timeout time until the state of the buffer underflow
  #   is reset, and if OnUnderflow is already set, this will be invoked to 
  #   return.
  #
  # @return Readout data
  #
  # @endif
  #
  #  DataType read()
  def read(self):
    self._rtcout.RTC_TRACE("DataType read()")

    if self._OnRead is not None:
      self._OnRead()
      self._rtcout.RTC_TRACE("OnRead called")

    if len(self._connectors) == 0:
      self._rtcout.RTC_DEBUG("no connectors")
      return self._value

    _val = copy.deepcopy(self._value)
    cdr = [_val]
    ret = self._connectors[0].read(cdr)


    if ret == OpenRTM_aist.DataPortStatus.PORT_OK:
      self._rtcout.RTC_DEBUG("data read succeeded")
      self._value = cdr[0]

      if self._OnReadConvert is not None:
        self._value = self._OnReadConvert(self._value)
        self._rtcout.RTC_DEBUG("OnReadConvert called")
        return self._value
      return self._value

    elif ret == OpenRTM_aist.DataPortStatus.BUFFER_EMPTY:
      self._rtcout.RTC_WARN("buffer empty")
      return self._value

    elif ret == OpenRTM_aist.DataPortStatus.BUFFER_TIMEOUT:
      self._rtcout.RTC_WARN("buffer read timeout")
      return self._value

    self._rtcout.RTC_ERROR("unknown retern value from buffer.read()")
    return self._value


  ##
  # @if jp
  #
  # @brief バインドされた変数に InPort バッファの最新値を読み込む
  #
  # バインドされたデータに InPort の最新値を読み込む。
  # コンストラクタで変数と InPort がバインドされていなければならない。
  # このメソッドはポリモーフィックに使用される事を前提としているため、
  # 型に依存しない引数、戻り値となっている。
  #
  # @param self
  #
  # @else
  #
  # @brief Read into bound T-type data from current InPort
  #
  # @endif
  def update(self):
    self.read()


  ##
  # @if jp
  #
  # @brief InPort バッファへデータ読み込み時のコールバックの設定
  #
  # InPort が持つバッファからデータが読み込まれる直前に呼ばれるコールバック
  # オブジェクトを設定する。
  # 
  # @param self
  # @param on_read 設定対象コールバックオブジェクト
  #
  # @else
  #
  # @endif
  def setOnRead(self, on_read):
    self._OnRead = on_read


  ##
  # @if jp
  #
  # @brief InPort バッファへデータ読み出し時のコールバックの設定
  #
  # InPort が持つバッファからデータが読み出される際に呼ばれるコールバック
  # オブジェクトを設定する。コールバックオブジェクトの戻り値がread()メソッド
  # の呼出結果となる。
  # 
  # @param self
  # @param on_rconvert 設定対象コールバックオブジェクト
  #
  # @else
  #
  # @endif
  def setOnReadConvert(self, on_rconvert):
    self._OnReadConvert = on_rconvert
