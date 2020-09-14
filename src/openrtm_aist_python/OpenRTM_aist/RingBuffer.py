#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file RingBuffer.py
# @brief Defautl Buffer class
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

import threading
import OpenRTM_aist


##
# @if jp
# @class RingBuffer
# @brief リングバッファ実装クラス
# 
# 指定した長さのリング状バッファを持つバッファ実装クラス。
# バッファ全体にデータが格納された場合、以降のデータは古いデータから
# 順次上書きされる。
# 従って、バッファ内には直近のバッファ長分のデータのみ保持される。
#
# 注)現在の実装では、一番最後に格納したデータのみバッファから読み出し可能
#
# @param DataType バッファに格納するデータ型
#
# @since 0.4.0
#
# @else
#
# @endif
class RingBuffer(OpenRTM_aist.BufferBase):
  """
  """

  RINGBUFFER_DEFAULT_LENGTH = 8

  ##
  # @if jp
  #
  # @brief コンストラクタ
  # 
  # コンストラクタ
  # 指定されたバッファ長でバッファを初期化する。
  #
  # @param length バッファ長
  # 
  # @else
  #
  # @brief Constructor
  # 
  # Constructor.
  # Initialize the buffer by specified buffer length.
  # However, if the specified length is less than two, the buffer should
  # be initialized by two in length.
  #
  # @param length Buffer length
  # 
  # @endif
  #
  # @endif
  def __init__(self, length=RINGBUFFER_DEFAULT_LENGTH):
    self._overwrite = True
    self._readback = True
    self._timedwrite = False
    self._timedread  = False
    self._wtimeout = OpenRTM_aist.TimeValue(1,0)
    self._rtimeout = OpenRTM_aist.TimeValue(1,0)
    self._length   = length
    self._wpos = 0
    self._rpos = 0
    self._fillcount = 0
    self._wcount = 0
    self._buffer = [None for i in range(self._length)]
    self._pos_mutex = threading.RLock()
    self._full_mutex = threading.RLock()
    self._empty_mutex = threading.RLock()
    self._full_cond = threading.Condition(self._full_mutex)
    self._empty_cond = threading.Condition(self._empty_mutex)


    self.reset()


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
  #
  # void init(const coil::Properties& prop)
  def init(self, prop):
    self.__initLength(prop)
    self.__initWritePolicy(prop)
    self.__initReadPolicy(prop)


  ##
  # @if jp
  #
  # @brief バッファ長を取得する
  # 
  # バッファ長を取得する。
  #
  # @param self
  # 
  # @return バッファ長
  # 
  # @else
  #
  # @brief Get the buffer length
  #
  # @endif
  #
  # size_t length(void) const
  def length(self, n = None):
    if n is None:
      guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
      return self._length

    if n < 1:
      return OpenRTM_aist.BufferStatus.NOT_SUPPORTED

    self._buffer = [None for i in range(n)]
    self._length = n
    self.reset()
    return OpenRTM_aist.BufferStatus.BUFFER_OK

  
  ##
  # @if jp
  #
  # @brief バッファの状態をリセットする
  # 
  # バッファの読み出しポインタと書き込みポインタの位置をリセットする。
  # この実装では BUFFER_OK しか返さない。
  # 
  # @return BUFFER_OK: 正常終了
  #         NOT_SUPPORTED: リセット不可能
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
  # ReturnCode reset()
  def reset(self):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    self._fillcount = 0
    self._wcount = 0
    self._wpos = 0
    self._rpos = 0
    return OpenRTM_aist.BufferStatus.BUFFER_OK


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
  # 
  # DataType* wptr(long int n = 0) 
  def wptr(self, n = 0):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    return self._buffer[(self._wpos + n + self._length) % self._length]

    
  ##
  # @if jp
  #
  # @brief 書込みポインタを進める
  # 
  # 現在の書き込み位置のポインタを n 個進める。
  # 書き込み可能な要素数以上の数値を指定した場合、PRECONDITION_NOT_MET
  # を返す。
  # 
  # @param  n 書込みポインタ + n の位置のポインタ 
  # @return BUFFER_OK:            正常終了
  #         PRECONDITION_NOT_MET: n > writable()
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
  # ReturnCode advanceWptr(long int n = 1)
  def advanceWptr(self, n = 1):
    # n > 0 :
    #     n satisfies n <= writable elements
    #                 n <= m_length - m_fillcout
    # n < 0 : -n = n'
    #     n satisfies n'<= readable elements
    #                 n'<= m_fillcount
    #                 n >= - m_fillcount
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    if (n > 0 and n > (self._length - self._fillcount)) or \
          (n < 0 and n < (-self._fillcount)):
      return OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET

    self._wpos = (self._wpos + n + self._length) % self._length
    self._fillcount += n
    return OpenRTM_aist.BufferStatus.BUFFER_OK


  ##
  # @if jp
  #
  # @brief バッファにデータを書き込む
  # 
  # バッファにデータを書き込む。書き込みポインタの位置は変更されない。
  # この実装では常に BUFFER_OK を返す。
  # 
  # @param value 書き込み対象データ
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
  # @return BUFFER_OK:    Successful
  #         BUFFER_ERROR: Failed
  #
  # @endif
  #
  # ReturnCode put(const DataType& value)
  def put(self, value):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    self._buffer[self._wpos] = value
    return OpenRTM_aist.BufferStatus.BUFFER_OK
    
  ##
  # @if jp
  #
  # @brief バッファに書き込む
  # 
  # 引数で与えられたデータをバッファに書き込む。
  #
  # 第2引数(sec)、第3引数(nsec)が指定されていない場合、バッファフル
  # 時の書込みモード (overwrite, do_nothing, block) は init() で設定
  # されたモードに従う。
  #
  # 第2引数(sec) に引数が指定された場合は、init() で設定されたモード
  # に関わらず、block モードとなり、バッファがフル状態であれば指定時
  # 間まち、タイムアウトする。第3引数(nsec)は指定されない場合0として
  # 扱われる。タイムアウト待ち中に、読み出しスレッド側でバッファから
  # 読み出せば、ブロッキングは解除されデータが書き込まれる。
  #
  # 書き込み時にバッファが空(empty)状態で、別のスレッドがblockモード
  # で読み出し待ちをしている場合、signalを発行して読み出し側のブロッ
  # キングが解除される。
  # 
  # @param value 書き込み対象データ
  # @param sec   タイムアウト時間 sec  (default -1: 無効)
  # @param nsec  タイムアウト時間 nsec (default 0)
  # @return BUFFER_OK            正常終了
  #         BUFFER_FULL          バッファがフル状態
  #         TIMEOUT              書込みがタイムアウトした
  #         PRECONDITION_NOT_MET 設定異常
  # 
  # @else
  #
  # @brief Write data into the buffer
  # 
  # Write data which is given argument into the buffer.
  #
  # @param value Target data for writing
  #
  # @return Writing result (Always true: writing success is returned)
  # 
  # @endif
  # 
  # ReturnCode write(const DataType& value,
  #                  long int sec = -1, long int nsec = 0)
  def write(self, value, sec = -1, nsec = 0):
    try:
      self._full_cond.acquire()
      if self.full():
        timedwrite = self._timedwrite # default is False
        overwrite  = self._overwrite  # default is True

        if not (sec < 0): # if second arg is set -> block mode
          timedwrite = True
          overwrite  = False

        if overwrite and not timedwrite:       # "overwrite" mode
          self.advanceRptr()

        elif not overwrite and not timedwrite: # "do_nothiong" mode
          self._full_cond.release()
          return OpenRTM_aist.BufferStatus.BUFFER_FULL

        elif not overwrite and timedwrite:     # "block" mode

          if sec < 0:
            sec = self._wtimeout.sec()
            nsec = self._wtimeout.usec() * 1000

          # true: signaled, false: timeout
          if not self._full_cond.wait(sec + (nsec/1000000000.0)):
            self._full_cond.release()
            return OpenRTM_aist.BufferStatus.TIMEOUT

        else: # unknown condition
          self._full_cond.release()
          return OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET
      
      self._full_cond.release()

      self.put(value)
      
      self._empty_cond.acquire()
      empty = self.empty()
      if empty:
        self.advanceWptr(1)
        self._empty_cond.notify()
      else:
        self.advanceWptr(1)
      self._empty_cond.release()

      return OpenRTM_aist.BufferStatus.BUFFER_OK
    except:
      return OpenRTM_aist.BufferStatus.BUFFER_OK

    
  ##
  # @if jp
  #
  # @brief バッファに書込み可能な要素数
  # 
  # バッファに書込み可能な要素数を返す。
  # 
  # @return 書き込み可能な要素数
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
  #
  # size_t writable() const
  def writable(self):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    return self._length - self._fillcount
    

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
  #
  # bool full(void) const
  def full(self):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    return self._length == self._fillcount
    

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
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    return self._buffer[(self._rpos + n + self._length) % self._length]

    
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
  # 
  # DataType* rptr(long int n = 0)
  def advanceRptr(self, n = 1):
    # n > 0 :
    #     n satisfies n <= readable elements
    #                 n <= m_fillcout 
    # n < 0 : -n = n'
    #     n satisfies n'<= m_length - m_fillcount
    #                 n >= m_fillcount - m_length
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    if (n > 0 and n > self._fillcount) or \
          (n < 0 and n < (self._fillcount - self._length)):
      return OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET

    self._rpos = (self._rpos + n + self._length) % self._length
    self._fillcount -= n
    return OpenRTM_aist.BufferStatus.BUFFER_OK


    
  ##
  # @if jp
  #
  # @brief バッファからデータを読み出す
  # 
  # バッファからデータを読みだす。読み出しポインタの位置は変更されない。
  # 
  # @param value 読み出しデータ
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
  #
  # ReturnCode get(DataType& value)
  def get(self, value=None):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    if value is None:
      return self._buffer[self._rpos]

    value[0] = self._buffer[self._rpos]
    return OpenRTM_aist.BufferStatus.BUFFER_OK
    
    
  ##
  # @if jp
  #
  # @brief バッファから読み出す
  # 
  # バッファに格納されたデータを読み出す。
  #
  # 第2引数(sec)、第3引数(nsec)が指定されていない場合、バッファ空状
  # 態での読み出しモード (readback, do_nothing, block) は init() で設
  # 定されたモードに従う。
  #
  # 第2引数(sec) に引数が指定された場合は、init() で設定されたモード
  # に関わらず、block モードとなり、バッファが空状態であれば指定時間
  # 待ち、タイムアウトする。第3引数(nsec)は指定されない場合0として扱
  # われる。タイムアウト待ち中に、書込みスレッド側でバッファへ書込み
  # があれば、ブロッキングは解除されデータが読みだされる。
  #
  # 読み出し時にバッファが空(empty)状態で、別のスレッドがblockモード
  # で書込み待ちをしている場合、signalを発行して書込み側のブロッキン
  # グが解除される。
  # 
  # @param value(list) 読み出し対象データ
  # @param sec   タイムアウト時間 sec  (default -1: 無効)
  # @param nsec  タイムアウト時間 nsec (default 0)
  # @return BUFFER_OK            正常終了
  #         BUFFER_EMPTY         バッファがフル状態
  #         TIMEOUT              書込みがタイムアウトした
  #         PRECONDITION_NOT_MET 設定異常
  # 
  # @else
  #
  # @brief Readout data from the buffer
  # 
  # Readout data stored into the buffer.
  # 
  # @param value(list) Readout data
  #
  # @return Readout result (Always true: readout success is returned)
  # 
  # @endif
  #
  # ReturnCode read(DataType& value,
  #                 long int sec = -1, long int nsec = 0)
  def read(self, value, sec = -1, nsec = 0):
    self._empty_cond.acquire()
      
    if self.empty():
      timedread = self._timedread
      readback  = self._readback

      if not (sec < 0):  # if second arg is set -> block mode
        timedread = True
        readback  = False
        sec = self._rtimeout.sec()
        nsec = self._rtimeout.usec() * 1000

      if readback and  not timedread:      # "readback" mode
        if not self._wcount > 0:
          self._empty_cond.release()
          return OpenRTM_aist.BufferStatus.BUFFER_EMPTY
        self.advanceRptr(-1)

      elif not readback and not timedread: # "do_nothiong" mode
        self._empty_cond.release()
        return OpenRTM_aist.BufferStatus.BUFFER_EMPTY

      elif not readback and timedread:     # "block" mode
        if sec < 0:
          sec = self._rtimeout.sec()
          nsec = self._rtimeout.usec() * 1000
        #  true: signaled, false: timeout
        if not self._empty_cond.wait(sec + (nsec/1000000000.0)):
          self._empty_cond.release()
          return OpenRTM_aist.BufferStatus.TIMEOUT

      else:                              # unknown condition
        self._empty_cond.release()
        return OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET
      
    self._empty_cond.release()

    val = self.get()

    if len(value) > 0:
      value[0] = val
    else:
      value.append(val)

    self._full_cond.acquire()
    full_ = self.full()

    if full_:
      self.advanceRptr()
      self._full_cond.notify()
    else:
      self.advanceRptr()

    self._full_cond.release()


    return OpenRTM_aist.BufferStatus.BUFFER_OK

    
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
  #
  # size_t readable() const
  def readable(self):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    return self._fillcount
    

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
  #
  # bool empty(void) const
  def empty(self):
    guard = OpenRTM_aist.ScopedLock(self._pos_mutex)
    return self._fillcount == 0

    
  ## void initLength(const coil::Properties& prop)
  def __initLength(self, prop):
    if prop.getProperty("length"):
      n = [0]
      if OpenRTM_aist.stringTo(n, prop.getProperty("length")):
        n = n[0]
        if n > 0:
          self.length(n)

    
  ## void initWritePolicy(const coil::Properties& prop)
  def __initWritePolicy(self, prop):
    policy = OpenRTM_aist.normalize([prop.getProperty("write.full_policy")])

    if policy == "overwrite":
      self._overwrite  = True
      self._timedwrite = False
    
    elif policy == "do_nothing":
      self._overwrite  = False
      self._timedwrite = False

    elif policy == "block":
      self._overwrite  = False
      self._timedwrite = True

      tm = [0.0]
      if OpenRTM_aist.stringTo(tm, prop.getProperty("write.timeout")):
        tm = tm[0]
        if not (tm < 0):
          self._wtimeout.set_time(tm)

    
  ## void initReadPolicy(const coil::Properties& prop)
  def __initReadPolicy(self, prop):
    policy = prop.getProperty("read.empty_policy")

    if policy == "readback":
      self._readback  = True
      self._timedread = False

    elif policy == "do_nothing":
      self._readback  = False
      self._timedread = False

    elif policy == "block":
      self._readback  = False
      self._timedread = True
      tm = [0.0]
      if OpenRTM_aist.stringTo(tm, prop.getProperty("read.timeout")):
        self._rtimeout.set_time(tm[0])
