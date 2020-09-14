#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ExtTrigExecutionContext.py
# @brief ExtTrigExecutionContext class
# @date $Date: 2007/09/06$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2007-2008
#    Task-intelligence Research Group,
#    Intelligent Systems Research Institute,
#    National Institute of
#        Advanced Industrial Science and Technology (AIST), Japan
#    All rights reserved.



import threading
import time

import OpenRTM_aist



##
# @if jp
# @class ExtTrigExecutionContext
# @brief ステップ実行が可能な ExecutionContext クラス
#
# １周期毎の実行が可能なPeriodic Sampled Data Processing(周期実行用)
# ExecutionContextクラス。
# 外部からのメソッド呼びだしによって時間を１周期づつ進めることができる。
#
# @since 0.4.0
#
# @else
# @class ExtTrigExecutionContext
# @endif
class ExtTrigExecutionContext(OpenRTM_aist.PeriodicExecutionContext):
  """
  """


  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  #
  # @else
  # @brief Constructor
  # @endif
  def __init__(self):
    OpenRTM_aist.PeriodicExecutionContext.__init__(self)
    self._worker = self.Worker()
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("rtobject.exttrig_ec")

  ##
  # @if jp
  # @brief 処理を１ステップ進める
  #
  # ExecutionContextの処理を１周期分進める。
  #
  # @param self
  #
  # @else
  #
  # @endif
  def tick(self):
    self._rtcout.RTC_TRACE("tick()")
    if not self._worker._cond.acquire():
      return
    self._worker._called = True
    self._worker._cond.notify()
    self._worker._cond.release()
    return

  ##
  # @if jp
  # @brief 各 Component の処理を呼び出す。
  # 
  # ExecutionContext に attach されている各 Component の処理を呼び出す。
  # 全 Component の処理を呼び出した後、次の呼出が発生するまで休止する。
  # 
  # @param self
  # 
  # @else
  # 
  # @endif
  # 
  def svc(self):
    self._rtcout.RTC_TRACE("svc()")
    flag = True

    while flag:
      sec_ = float(self._period.usec())/1000000.0
      self._worker._cond.acquire()
      while not self._worker._called and self._running:
        self._worker._cond.wait()
      if self._worker._called:
        self._worker._called = False
        for comp in self._comps:
          comp._sm.worker()

      self._worker._cond.release()
      flag = self._running

  ##
  # @if jp
  # @class Worker
  # @brief ExecutionContext 駆動クラス
  #
  # 実行処理に関する排他制御など、実際の処理を監視・制御するためのクラス。
  #
  # @since 0.4.0
  #
  # @else
  #
  # @endif
  class Worker:
    """
    """
    
    ##
    # @if jp
    # @brief コンストラクタ
    #
    # コンストラクタ
    #
    # @param self
    #
    # @else
    # @brief Constructor
    # @endif
    def __init__(self):
      self._mutex = threading.RLock()
      self._cond = threading.Condition(self._mutex)
      self._called = False



##
# @if jp
# @brief 当該 ExecutionContext 用Factoryクラスの登録。
#
# このExecutionContextを生成するFactoryクラスを
# ExecutionContext管理用ObjectManagerに登録する。
#
# @else
#
# @endif
def ExtTrigExecutionContextInit(manager):
  manager.registerECFactory("ExtTrigExecutionContext",
                            OpenRTM_aist.ExtTrigExecutionContext,
                            OpenRTM_aist.ECDelete)
