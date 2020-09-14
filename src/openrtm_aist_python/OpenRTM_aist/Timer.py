#!/usr/bin/env/python
# -*- coding: euc-jp -*-

##
# @file Timer.py
# @brief Timer class
# @date $Date: $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2007-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import time
import threading

import OpenRTM_aist


##
# @if jp
# @class Timer
# @brief Timerクラス
# 
# 登録されたリスナーのコールバック関数を、設定された周期で定期的に呼び出す。
#
# @since 0.4.0
#
# @else
#
# @class Timer
# @brief Timer class
# 
# Invoke the callback function of registered listener periodically
# at the set cycle.
#
# @since 0.4.0
#
# @endif
class Timer:
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  # 
  # コンストラクタ
  #
  # @param self
  # @param interval タイマ起動周期
  #
  # @else
  #
  # @brief Constructor
  # 
  # Constructor
  #
  # @param interval The interval of timer
  #
  # @endif
  def __init__(self, interval):
    self._interval = interval
    self._running  = False
    self._runningMutex = threading.RLock()
    self._tasks = []
    self._taskMutex = threading.RLock()
    self._thread = threading.Thread(target=self.run)
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
    self._running = False
    try:
      self.join()
    except:
      pass
    self._thread = None

  def join(self):
    try:
      self._thread.join()
      self._thread = threading.Thread(target=self.run)
    except:
      pass

    return

  ##
  # @if jp
  # @brief Timer 用のスレッド実行関数
  #
  # Timer 用のスレッド実行関数。
  # 登録されたリスナーのコールバック関数を呼び出す。
  #
  # @return 実行結果
  #
  # @else
  # @brief Thread execution function for Timer
  #
  # Thread execution function for Timer.
  # Invoke the callback function of registered listener.
  #
  # @return Execution result
  #
  # @endif
  def run(self):
    while self._running:
      self.invoke()
      if self._interval.tv_sec != 0:
        time.sleep(self._interval.tv_sec)
      elif self._interval.tv_usec:
        time.sleep(self._interval.tv_usec/1000000.0)
    return 0


  ##
  # @if jp
  # @brief Timer タスク開始
  #
  # Timer 用新規スレッドを生成し、処理を開始する。
  #
  # @param self
  #
  # @brief Start Timer task
  #
  # Create a new theread for Timer and start processing.
  #
  # @else
  #
  # @endif
  def start(self):
    guard = OpenRTM_aist.ScopedLock(self._runningMutex)
    if not self._running:
      self._running = True
      self._thread.start()
    return


  ##
  # @if jp
  # @brief Timer タスク停止
  #
  # @param self
  #
  # Timer タスクを停止する。
  #
  # @else
  #
  # @brief Stop Timer task
  #
  # Stop Timer task.
  #
  # @endif
  def stop(self):
    guard = OpenRTM_aist.ScopedLock(self._runningMutex)
    if self._running:
      self._running = False
      self.join()
    return


  ##
  # @if jp
  # @brief Timer タスク実行
  #
  # @param self
  #
  # 登録された各リスナの起動待ち時間からタイマ起動周期を減算する。
  # 起動待ち時間がゼロとなったリスナが存在する場合は、
  # コールバック関数を呼び出す。
  #
  # @else
  #
  # @brief Invoke Timer task
  #
  # Subtract the interval of timer from the waiting time for invocation
  # of each registered listener.
  # If the listener whose waiting time reached 0 exists, invoke the
  # callback function.
  #
  # @endif
  def invoke(self):
    guard = OpenRTM_aist.ScopedLock(self._taskMutex)
    for i in range(len(self._tasks)):
      self._tasks[i].remains = self._tasks[i].remains - self._interval
      if self._tasks[i].remains.sign() <= 0.0:
        self._tasks[i].remains = self._tasks[i].period
        self._tasks[i].listener.invoke()
    del guard
    return

  ##
  # @if jp
  # @brief リスナー登録
  #
  # 本 Timer から起動するコールバック関数用のリスナーを起動周期を指定して
  # 登録する。
  # 同一リスナーが既に登録済みの場合は、リスナーの起動周期を指定した値に
  # 更新する。
  #
  # @param self
  # @param listener 登録対象リスナー
  # @param tm リスナー起動周期
  #
  # @return 登録リスナー
  #
  # @else
  #
  # @brief Register listener
  #
  # Register the listener of callback function invoked from this Timer by
  # specifying the interval.
  # If the same listener has already been regiseterd, the value specified
  # the invocation interval of listener will be updated.
  # 
  #
  # @param listener Listener for the registration
  # @param tm The invocation interval of listener
  #
  # @return ID of the registerd listener
  #
  # @endif
  # ListenerId registerListener(ListenerBase* listener, TimeValue tm);
  def registerListener(self, listener, tm):
    guard = OpenRTM_aist.ScopedLock(self._taskMutex)
    for i in range(len(self._tasks)):
      if self._tasks[i].listener == listener:
        self._tasks[i].period = tm
        self._tasks[i].remains = tm
        return listener
    self._tasks.append(self.Task(listener, tm))
    return listener


  ##
  # @if jp
  # @brief リスナー登録
  #
  # コールバック対象オブジェクト、コールバック対象メソッドおよび起動周期を
  # 指定してリスナーを登録する。
  #
  # @param self
  # @param obj コールバック対象オブジェクト
  # @param cbf コールバック対象メソッド
  # @param tm リスナー起動周期
  #
  # @return 登録リスナー
  #
  # @else
  #
  # @brief Register listener
  #
  # Register listener by specifying the object for callback, the method
  # for callback and the invocation interval.
  #
  # @param obj Target object for callback
  # @param cbf Target method for callback
  # @param tm The invocation interval of listener
  #
  # @return ID of the registerd listener
  #
  #
  # @endif
  #  template <class ListenerClass>
  #  ListenerId registerListenerObj(ListenerClass* obj,
  #                                 void (ListenerClass::*cbf)(),
  #                                 TimeValue tm)
  def registerListenerObj(self, obj, cbf, tm):
    return self.registerListener(OpenRTM_aist.ListenerObject(obj, cbf), tm)


  ##
  # @if jp
  # @brief リスナー登録
  #
  # コールバック対象メソッドと起動周期を指定してリスナーを登録する。
  #
  # @param self
  # @param cbf コールバック対象メソッド
  # @param tm リスナー起動周期
  #
  # @return 登録リスナー
  #
  # @else
  #
  # @brief Register listener
  #
  # Register listener by specifying the method for callback and the
  # invocation interval.
  #
  # @param cbf Target method for callback
  # @param tm The invocation interval of listener
  #
  # @return ID of the registerd listener
  #
  # @endif
  # ListenerId registerListenerFunc(void (*cbf)(), TimeValue tm)
  def registerListenerFunc(self, cbf, tm):
    return self.registerListener(OpenRTM_aist.ListenerFunc(cbf), tm)


  ##
  # @if jp
  # @brief リスナー登録解除
  #
  # 指定したIDのリスナーの登録を解除する。
  # 指定したIDのリスナーが未登録の場合、false を返す。
  #
  # @param self
  # @param id 登録解除対象リスナーID
  #
  # @return 登録解除結果
  #
  # @else
  #
  # @brief Unregister listener
  #
  # Unregister the listener specified by ID.
  # If the listener specified by ID is not registerd, false will be returned.
  #
  # @param id ID of the unregisterd listener
  #
  # @return Unregistration result
  #
  # @endif
  # bool unregisterListener(ListenerId id);
  def unregisterListener(self, id):
    guard = OpenRTM_aist.ScopedLock(self._taskMutex)
    len_ = len(self._tasks)
    for i in range(len_):
      idx = (len_ - 1) - i
      if self._tasks[idx].listener == id:
        del self._tasks[idx]
        return True
    return False


  ##
  # @if jp
  # @class Task
  # @brief タスク管理用クラス
  # @else
  #
  # @endif
  class Task:
    def __init__(self, lb, tm):
      self.listener = lb
      self.period = tm
      self.remains = tm
      return
