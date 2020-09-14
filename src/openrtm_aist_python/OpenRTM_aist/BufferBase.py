#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file BufferBase.py
# @brief Buffer abstract class
# @date $Date: 2007/09/12 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
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
# @class BufferBase
# @brief BufferBase 抽象クラス
# 
# 種々のバッファのための抽象インターフェースクラス。
# 具象バッファクラスは、以下の関数の実装を提供しなければならない。
# 
# publicインターフェースとして以下のものを提供する。
#  - write(): バッファに書き込む
#  - read(): バッファから読み出す
#  - length(): バッファ長を返す
#  - isFull(): バッファが満杯である
#  - isEmpty(): バッファが空である
# 
# protectedインターフェースとして以下のものを提供する。
#  - put(): バッファにデータを書き込む
#  - get(): バッファからデータを読み出す
# 
# @since 0.4.0
# 
# @else
# 
# @class BufferBase
# @brief BufferBase abstract class
# 
# This is the abstract interface class for various Buffer.
# 
# @since 0.4.0
# 
# @endif
class BufferBase(OpenRTM_aist.BufferStatus):
  """
  """


  ##
  # @if jp
  # @brief バッファの設定
  #
  # Properties で与えられるプロパティにより、
  # バッファの設定を初期化する。
  # 使用できるオプションと意味は以下の通り
  #
  # - buffer.length:
  #     バッファの長さ。自然数以外の数値が指定されても無視される。す
  #     でにバッファが使用状態でも、長さが再設定されたのち、すべての
  #     ポインタが初期化される。
  #
  # - buffer.write.full_policy:
  #     上書きするかどうかのポリシー。
  #     overwrite (上書き), do_nothing (何もしない), block (ブロックする)
  #     block を指定した場合、次の timeout 値を指定すれば、指定時間後
  #     書き込み不可能であればタイムアウトする。
  #     デフォルトは  overwrite (上書き)。
  #
  # - buffer.write.timeout:
  #     タイムアウト時間を [sec] で指定する。デフォルトは 1.0 [sec]。
  #     1 sec -> 1.0, 1 ms -> 0.001, タイムアウトしない -> 0.0
  #
  # - buffer.read.empty_policy:
  #     バッファが空のときの読み出しポリシー。
  #     readback (最後の要素), do_nothing (何もしない), block (ブロックする)
  #     block を指定した場合、次の timeout 値を指定すれば、指定時間後
  #     読み出し不可能であればタイムアウトする。
  #     デフォルトは readback (最後の要素)。
  #
  # - buffer.read.timeout:
  #     タイムアウト時間 [sec] で指定する。デフォルトは 1.0 [sec]。
  #     1sec -> 1.0, 1ms -> 0.001, タイムアウトしない -> 0.0
  #
  # @else
  #
  # @endif
  def init(self, prop):
    pass


  ##
  # @if jp
  # 
  # @brief バッファの長さを取得する(サブクラス実装用)
  # 
  # バッファ長を取得する<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self 
  # 
  # @return バッファ長
  # 
  # @else
  # 
  # @brief Get the buffer length
  # 
  # @return buffer length
  # 
  # @endif
  def length(self):
    pass


  ##
  # @if jp
  #
  # @brief バッファの状態をリセットする
  # 
  # バッファの読み出しポインタと書き込みポインタの位置をリセットする。
  # 
  # @return BUFFER_OK: 正常終了
  #         NOT_SUPPORTED: バッファ長変更不可
  #         BUFFER_ERROR: 異常終了
  # 
  # @else
  #
  # @brief Get the buffer length
  #
  # Pure virtual function to get the buffer length.
  #
  # @return buffer length
  # 
  # @endif
  #
  def reset(self):
    pass


  ##
  # @if jp
  #
  # @brief バッファの現在の書込み要素のポインタ
  # 
  # バッファの現在の書込み要素のポインタまたは、n個先のポインタを返す
  # 
  # @param  n 書込みポインタ + n の位置のポインタ 
  # @return 書込み位置のポインタ
  # 
  # @else
  #
  # @brief Get the buffer length
  #
  # Pure virtual function to get the buffer length.
  #
  # @return buffer length
  # 
  # @endif
  def wptr(self, n=0):
    pass


  ##
  # @if jp
  #
  # @brief 書込みポインタを進める
  # 
  # 現在の書き込み位置のポインタを n 個進める。
  # 
  # @param  n 書込みポインタ + n の位置のポインタ 
  # @return BUFFER_OK: 正常終了
  #         BUFFER_ERROR: 異常終了
  # 
  # @else
  #
  # @brief Get the buffer length
  #
  # Pure virtual function to get the buffer length.
  #
  # @return buffer length
  # 
  # @endif
  def advanceWptr(self, n = 1):
    pass


  ##
  # @if jp
  # 
  # @brief バッファにデータを格納する(サブクラス実装用)
  # 
  # バッファへのデータ格納用関数<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self 
  # @param data 対象データ
  # 
  # @else
  # 
  # @brief Write data into the buffer
  # 
  # @endif
  def put(self, data):
    pass


  ##
  # @if jp
  # 
  # @brief バッファにデータを書き込む(サブクラス実装用)
  # 
  # バッファにデータを書き込む<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self 
  # @param value 書き込み対象データ
  # 
  # @return データ書き込み結果(true:書き込み成功，false:書き込み失敗)
  # 
  # @else
  # 
  # @brief Write data into the buffer
  # 
  # @endif
  def write(self, value, sec=-1, nsec=-1):
    pass


  ##
  # @if jp
  #
  # @brief バッファに書込み可能な要素数
  # 
  # バッファに書込み可能な要素数を返す。
  # 
  # @return 書き込み可能な要素数
  #
  # @return BUFFER_OK: 正常終了
  #         BUFFER_ERROR: 異常終了
  # 
  # @else
  #
  # @brief Write data into the buffer
  #
  # Pure virtual function to write data into the buffer.
  #
  # @param value Target data to write.
  #
  # @return Result of having written in data (true:Successful, false:Failed)
  #
  # @endif
  def writable(self):
    pass


  ##
  # @if jp
  #
  # @brief バッファfullチェック
  # 
  # バッファfullチェック用純粋仮想関数
  #
  # @return fullチェック結果(true:バッファfull，false:バッファ空きあり)
  # 
  # @else
  #
  # @brief Check on whether the buffer is full.
  #
  # Pure virtual function to check on whether the buffer is full.
  #
  # @return True if the buffer is full, else false.
  #
  # @endif
  def full(self):
    pass


  ##
  # @if jp
  #
  # @brief バッファの現在の読み出し要素のポインタ
  # 
  # バッファの現在の読み出し要素のポインタまたは、n個先のポインタを返す
  # 
  # @param  n 読み出しポインタ + n の位置のポインタ 
  # @return 読み出し位置のポインタ
  # 
  # @else
  #
  # @brief Get the buffer length
  #
  # Pure virtual function to get the buffer length.
  #
  # @return buffer length
  # 
  # @endif
  def rptr(self, n = 0):
    pass

  ##
  # @if jp
  #
  # @brief 読み出しポインタを進める
  # 
  # 現在の読み出し位置のポインタを n 個進める。
  # 
  # @param  n 読み出しポインタ + n の位置のポインタ 
  # @return BUFFER_OK: 正常終了
  #         BUFFER_ERROR: 異常終了
  # 
  # @else
  #
  # @brief Get the buffer length
  #
  # Pure virtual function to get the buffer length.
  #
  # @return buffer length
  # 
  # @endif
  def advanceRptr(self, n = 1):
    pass


  ##
  # @if jp
  # 
  # @brief バッファからデータを取得する(サブクラス実装用)
  # 
  # バッファに格納されたデータ取得用関数<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self 
  # 
  # @return 取得データ
  # 
  # @else
  # 
  # @brief Get data from the buffer
  # 
  # @endif
  def get(self):
    pass


  ##
  # @if jp
  # 
  # @brief バッファからデータを読み出す(サブクラス実装用)
  # 
  # バッファからデータを読み出す<BR>
  # ※サブクラスでの実装参照用
  # 
  # @param self 
  # @param value 読み出しデータ
  # 
  # @return データ読み出し結果(true:読み出し成功，false:読み出し失敗)
  # 
  # @else
  # 
  # @brief Read data from the buffer
  # 
  # @endif
  def read(self, value, sec = -1, nsec = -1):
    pass


  ##
  # @if jp
  #
  # @brief バッファから読み出し可能な要素数
  # 
  # バッファから読み出し可能な要素数を返す。
  # 
  # @return 読み出し可能な要素数
  #
  # @return BUFFER_OK: 正常終了
  #         BUFFER_ERROR: 異常終了
  # 
  # @else
  #
  # @brief Write data into the buffer
  #
  # Pure virtual function to write data into the buffer.
  #
  # @param value Target data to write.
  #
  # @return Result of having written in data (true:Successful, false:Failed)
  #
  # @endif
  def readable(self):
    pass


  ##
  # @if jp
  #
  # @brief バッファemptyチェック
  # 
  # バッファemptyチェック用純粋仮想関数
  #
  # @return emptyチェック結果(true:バッファempty，false:バッファデータあり)
  # 
  # @else
  #
  # @brief Check on whether the buffer is empty.
  #
  # Pure virtual function to check on whether the buffer is empty.
  #
  # @return True if the buffer is empty, else false.
  #
  # @endif
  def empty(self):
    pass



##
# @if jp
# @class NullBuffer
# @brief ダミーバッファ実装クラス
# 
# バッファ長が１固定のダミーバッファ実装クラス。
# 
# @param DataType バッファに格納するデータ型
# 
# @since 0.4.0
# 
# @else
# 
# @endif
class NullBuffer(BufferBase):
  """
  """



  ##
  # @if jp
  # 
  # @brief コンストラクタ
  # 
  # コンストラクタ
  # バッファ長を１(固定)で初期化する。
  # 
  # @param self 
  # @param size バッファ長(デフォルト値:None，ただし無効)
  # 
  # @else
  # 
  # @endif
  def __init__(self, size=None):
    self._length = 1
    self._data = None
    self._is_new = False
    self._inited = False


  ##
  # @if jp
  # 
  # @brief バッファ長(１固定)を取得する
  # 
  # バッファ長を取得する。(常に１を返す。)
  # 
  # @param self 
  # 
  # @return バッファ長(１固定)
  # 
  # @else
  # 
  # @brief Get the buffer length
  # 
  # @return buffer length(always 1)
  # 
  # @endif
  def length(self):
    return 1


  ##
  # @if jp
  # 
  # @brief バッファにデータを書き込む
  # 
  # 引数で与えられたデータをバッファに書き込む。
  # 
  # @param self 
  # @param value 書き込み対象データ
  # 
  # @return データ書き込み結果(true:書き込み成功，false:書き込み失敗)
  # 
  # @else
  # 
  # @brief Write data into the buffer
  # 
  # @endif
  def write(self, value, sec=-1, nsec=-1):
    self.put(value)
    return True


  ##
  # @if jp
  # 
  # @brief バッファからデータを読み出す
  # 
  # バッファに格納されたデータを読み出す。
  # 
  # @param self 
  # @param value 読み出したデータ
  # 
  # @return データ読み出し結果(true:読み出し成功，false:読み出し失敗)
  # 
  # @else
  # 
  # @brief Read data from the buffer
  # 
  # @endif
  def read(self, value):
    if not self._inited:
      return False
    value[0] = self.get()
    return True


  ##
  # @if jp
  # 
  # @brief バッファfullチェック
  # 
  # バッファfullをチェックする。(常にfalseを返す。)
  # 
  # @param self 
  # 
  # @return fullチェック結果(常にfalse)
  # 
  # @else
  # 
  # @brief Always false.
  # 
  # @endif
  def isFull(self):
    return False


  ##
  # @if jp
  # 
  # @brief バッファemptyチェック
  # 
  # バッファemptyをチェックする。(常にfalseを返す。)
  # ※要確認
  # 
  # @param self 
  # 
  # @return emptyチェック結果(常にfalse)
  # 
  # @else
  # 
  # @brief Always false.
  # 
  # @endif
  def isEmpty(self):
    return False


  ##
  # @if jp
  # 
  # @brief 最新データか確認する
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
  def isNew(self):
    return self._is_new


  ##
  # @if jp
  # 
  # @brief バッファにデータを格納
  # 
  # 引数で与えられたデータをバッファに格納する。
  # 
  # @param self 
  # @param data 対象データ
  # 
  # @else
  # 
  # @brief Write data into the buffer
  # 
  # @endif
  def put(self, data):
    self._data = data
    self._is_new = True
    self._inited = True


  ##
  # @if jp
  # 
  # @brief バッファからデータを取得する
  # 
  # バッファに格納されたデータを取得する。
  # 
  # @param self 
  # 
  # @return 取得データ
  # 
  # @else
  # 
  # @brief Get data from the buffer
  # 
  # @endif
  def get(self):
    self._is_new = False
    return self._data


  ##
  # @if jp
  # 
  # @brief 次に書き込むバッファへの参照を取得する
  # 
  # 書き込みバッファへの参照を取得する。
  # 本バッファ実装ではバッファ長は固定で１であるため，
  # 常に同じ位置への参照を返す。
  # 
  # @param self 
  # 
  # @return 次の書き込み対象バッファへの参照(固定)
  # 
  # @else
  # 
  # @brief Get the buffer's reference to be written the next
  # 
  # @endif
  def getRef(self):
    return self._data
