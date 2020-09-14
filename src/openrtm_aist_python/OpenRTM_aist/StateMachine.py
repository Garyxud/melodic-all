#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file StateMachine.py
# @brief State machine template class
# @date $Date: 2007/08/30$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import threading

import OpenRTM_aist
import RTC


##
# @if jp
# @class StateHolder
# @brief 状態保持用クラス
# 
# 状態を保持するためのホルダークラス。
# 現在の状態と、１つ前の状態、遷移予定の状態を保持する。
#
# @param State 保持する状態の型
#
# @since 0.4.0
#
# @else
#
# @endif
class StateHolder:
  def __init__(self):
    self.curr = None
    self.prev = None
    self.next = None


##
# @if jp
#
# @class StateMachine
#
# @brief 状態マシンクラス
#
# StateMachine クラスは状態マシンを実現するクラスである。
#
# 例: ActiveObjectは状態マシンを持つアクティブオブジェクトであるとする。
# 状態は3状態 INACTIVE, ACTIVE, ERROR があり、各状態でのEntryやExit動作を
# 定義したいとすると、以下のように実現される。
# <pre>
# class ActiveObject:
#   class MyState:
#     INACTIVE, ACTIVE, ERROR = range(3)
# 
#   def __init__(self):
#     m_sm = StateMachine(3)
#     m_sm.setNOP(nullAction)
#     m_sm.setListener(self)
# 
#     m_sm.setExitAction(MyState.INACTIVE, self.inactiveExit)
#       : 
#     m_sm.setPostDoAction(MyState.ERROR, self.errorPostDo)
#     m_sm.setTransitionAction(self.transition); 
# 
#   def nullAction(myStates):
#     pass
#   def inactiveExit(myStates):
#     pass
#     : 
#   def errorPostDo(myStates):
#     pass
#   def transition(myStates:
#     pass
# </pre>
# 状態を持たせたいクラスは以下の条件を満たすように実装しなければならない。
# <ol>
# <li> 内部クラスで状態を定義
# <li> StateMachine のコンストラクタ引数は状態の数
# <li> 以下のアクション関数を(Return _function_name_(States)) の関数として設定
# <ol>
#  <li> 何もしない関数を必ず定義し、setNOP で与えなければならない
#  <li> 各状態毎に, set(Entry|PreDo|Do|PostDo|Exit)Action でアクションを設定
#  <li> 状態遷移時のアクションを setTransitionAction() で設定。
# </ol>
# <li> 遷移時のアクションは、与えられた現在状態、次状態、前状態を元に、
#   ユーザが実装しなければならない。
# <li> 状態の変更は goTo() で、状態のチェックは isIn(state) で行う。
# <li> goTo()は次状態を強制的にセットする関数であり、遷移の可否は、
#   ユーザが現在状態を取得し判断するロジックを実装しなければならない。
# </ol>
#
# このクラスは、一つの状態に対して、
# <ul>
# <li> Entry action
# <li> PreDo action
# <li> Do action
# <li> PostDo action
# <li> Exit action
# </ul>
# 5つのアクションが定義することができる。
# Transition action はあらゆる状態間遷移で呼び出されるアクションで、
# その振る舞いはユーザが定義しなければならない。
# 
# このクラスは以下のようなタイミングで各アクションが実行される。
#
# <ul>
# <li> 状態が変更され(A->B)状態が遷移する場合 <br>
# (A:Exit)->|(状態更新:A->B)->(B:Entry)->(B:PreDo)->(B:Do)->(B:PostDo)
#
# <li> 状態が変更されず、B状態を維持する場合 (|はステップの区切りを表す)<br>
# (B(n-1):PostDo)->|(B(n):PreDo)->(B(n):Do)->(B(n):PostDo)->|(B(n+1):PreDo)<br>
# PreDo, Do, PostDo が繰り返し実行される。
#
# <li> 自己遷移する場合 <br>
# (B(n-1):PostDo)->(B(n-1):Exit)->|(B(n):Entry)->(B(n):PreDo) <br>
# 一旦 Exit が呼ばれた後、Entry が実行され、以降は前項と同じ動作をする。
# </ul>
#
# @since 0.4.0
#
# @else
#
# @brief
#
# @endif
class StateMachine:
  """
  """

  state_array = (RTC.CREATED_STATE,
                 RTC.INACTIVE_STATE,
                 RTC.ACTIVE_STATE,
                 RTC.ERROR_STATE)

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param num_of_state ステートマシン中の状態数
  #
  # @else
  # @brief Constructor
  # @endif
  def __init__(self, num_of_state):
    self._num = num_of_state
    self._entry  = {}
    self._predo  = {}
    self._do     = {}
    self._postdo = {}
    self._exit   = {}

    self.setNullFunc(self._entry,  None)
    self.setNullFunc(self._do,     None)
    self.setNullFunc(self._exit,   None)
    self.setNullFunc(self._predo,  None)
    self.setNullFunc(self._postdo, None)
    self._transit = None
    self._mutex = threading.RLock()


  ##
  # @if jp
  # @brief NOP関数を登録する
  #
  # NOP関数(何もしない関数)を登録する。
  #
  # @param self
  # @param call_back コールバック関数
  #
  # @else
  # @brief Set NOP function
  # @endif
  def setNOP(self, call_back):
    self.setNullFunc(self._entry,  call_back)
    self.setNullFunc(self._do,     call_back)
    self.setNullFunc(self._exit,   call_back)
    self.setNullFunc(self._predo,  call_back)
    self.setNullFunc(self._postdo, call_back)
    self._transit = call_back


  ##
  # @if jp
  # @brief Listener オブジェクトを登録する
  #
  # 各種アクション実行時に呼び出される Listener オブジェクトを登録する。
  #
  # @param self
  # @param listener Listener オブジェクト
  #
  # @else
  # @brief Set Listener Object
  # @endif
  def setListener(self, listener):
    self._listener = listener


  ##
  # @if jp
  # @brief Entry action 関数を登録する
  #
  # 各状態に入った際に実行される Entry action 用コールバック関数を登録する。
  #
  # @param self
  # @param state 登録対象状態
  # @param call_back Entry action 用コールバック関数
  #
  # @return アクション実行結果
  #
  # @else
  # @brief Set Entry action function
  # @endif
  def setEntryAction(self, state, call_back):
    if self._entry.has_key(state):
      self._entry[state] = call_back
    else:
      self._entry.setdefault(state, call_back)
    return True


  ##
  # @if jp
  # @brief PreDo action 関数を登録する
  #
  # 各状態内で実行される PreDo action 用コールバック関数を登録する。
  #
  # @param self
  # @param state 登録対象状態
  # @param call_back PreDo action 用コールバック関数
  #
  # @return アクション実行結果
  #
  # @else
  # @brief Set PreDo action function
  # @endif
  def setPreDoAction(self, state, call_back):
    if self._predo.has_key(state):
      self._predo[state] = call_back
    else:
      self._predo.setdefault(state, call_back)
    return True


  ##
  # @if jp
  # @brief Do action 関数を登録する
  #
  # 各状態内で実行される Do action 用コールバック関数を登録する。
  #
  # @param self
  # @param state 登録対象状態
  # @param call_back Do action 用コールバック関数
  #
  # @return アクション実行結果
  #
  # @else
  # @brief Set Do action function
  # @endif
  def setDoAction(self, state, call_back):
    if self._do.has_key(state):
      self._do[state] = call_back
    else:
      self._do.setdefault(state, call_back)
    return True


  ##
  # @if jp
  # @brief PostDo action 関数を登録する
  #
  # 各状態内で実行される PostDo action 用コールバック関数を登録する。
  #
  # @param self
  # @param state 登録対象状態
  # @param call_back PostDo action 用コールバック関数
  #
  # @return アクション実行結果
  #
  # @else
  # @brief Set PostDo action function
  # @endif
  def setPostDoAction(self, state, call_back):
    if self._postdo.has_key(state):
      self._postdo[state] = call_back
    else:
      self._postdo.setdefault(state, call_back)
    return True


  ##
  # @if jp
  # @brief Exit action 関数を登録する
  #
  # 各状態内で実行される Exit action 用コールバック関数を登録する。
  #
  # @param self
  # @param state 登録対象状態
  # @param call_back Exit action 用コールバック関数
  #
  # @return アクション実行結果
  #
  # @else
  # @brief Set Exit action function
  # @endif
  def setExitAction(self, state, call_back):
    if self._exit.has_key(state):
      self._exit[state] = call_back
    else:
      self._exit.setdefault(state, call_back)
    return True


  ##
  # @if jp
  # @brief State transition action 関数を登録する
  #
  # 状態遷移時に実行される State transition action 用コールバック関数を
  # 登録する。
  #
  # @param self
  # @param call_back State transition 用コールバック関数
  #
  # @return アクション実行結果
  #
  # @else
  # @brief Set state transition action function
  # @endif
  def setTransitionAction(self, call_back):
    self._transit = call_back
    return True


  ##
  # @if jp
  # @brief 初期状態をセットする
  #
  # ステートマシンの初期状態を設定する。
  #
  # @param self
  # @param states 初期状態
  #
  # @else
  # @brief Set Exit action function
  # @endif
  def setStartState(self, states):
    self._states = StateHolder()
    self._states.curr = states.curr
    self._states.prev = states.prev
    self._states.next = states.next


  ##
  # @if jp
  # @brief 状態を取得する
  #
  # 状態情報を取得する。
  # 現在の状態、１つ前の状態、遷移予定の状態を取得することができる。
  #
  # @param self
  #
  # @return 状態情報
  #
  # @else
  # @brief Get state machine's status
  # @endif
  def getStates(self):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    return self._states


  ##
  # @if jp
  # @brief 現在の状態を取得する
  #
  # 現在の状態を取得する。
  #
  # @param self
  #
  # @return 現在の状態
  #
  # @else
  # @brief Get current state
  # @endif
  def getState(self):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    return self._states.curr


  ##
  # @if jp
  # @brief 現在状態を確認
  #
  # 現在の状態が、引数で指定した状態と一致するか確認する。
  #
  # @param self
  # @param state 確認対象状態
  #
  # @return 状態確認結果
  #
  # @else
  # @brief Evaluate current status
  # @endif
  def isIn(self, state):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    if self._states.curr == state:
      return True
    else:
      return False


  ##
  # @if jp
  # @brief 状態を遷移
  #
  # 指定した状態に状態を遷移する。
  # 本関数は次状態を強制的にセットする関数である。
  # このため、遷移の可否は、ユーザが現在状態を取得し判断するロジックを
  # 実装しなければならない。
  # 遷移先が現在の状態と同じ場合には、自己遷移フラグをセットする。
  #
  # @param self
  # @param state 遷移先状態
  #
  # @else
  # @brief Change status
  # @endif
  def goTo(self, state):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    self._states.next = state


  ##
  # @if jp
  # @brief 駆動関数
  #
  # ステートマシンの駆動関数。
  # 実際の状態遷移および状態遷移発生時の各アクションの呼びだしを実行する。
  #
  # @param self
  #
  # @else
  # @brief Worker function
  # @endif
  def worker(self):
    states = StateHolder()
    self.sync(states)

    # If no state transition required, execute set of do-actions
    if states.curr == states.next:
      # pre-do
      if self._predo[states.curr]:
        self._predo[states.curr](states)
      if self.need_trans():
        return

      # do
      if self._do[states.curr]:
        self._do[states.curr](states)
      if self.need_trans():
        return

      # post-do
      if self._postdo[states.curr]:
        self._postdo[states.curr](states)
    # If state transition required, exit current state and enter next state
    else:
      if self._exit[states.curr]:
        self._exit[states.curr](states)
      self.sync(states)

      # If state transition still required, move to the next state
      if states.curr != states.next:
        states.curr = states.next
        if self._entry[states.curr]:
          self._entry[states.curr](states)
        self.update_curr(states.curr)


  ##
  # @if jp
  # @brief NOP関数を設定
  #
  # NOP関数(何もしない関数)を登録する。
  #
  # @param self
  # @param s コールバック関数設定先
  # @param nullfunc コールバック関数(NOP関数)
  #
  # @else
  # @brief Worker function
  # @endif
  def setNullFunc(self, s, nullfunc):
    for i in range(self._num):
      if s.has_key(StateMachine.state_array[i]):
        s[StateMachine.state_array[i]] = nullfunc
      else:
        s.setdefault(StateMachine.state_array[i], nullfunc)


  ##
  # @if jp
  # @brief 状態の同期処理
  #
  # @param self
  # @param states OpenRTM_aist.StateHolder<RTC.LifeCycleState>
  #
  # @else
  # @endif
  def sync(self, states):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    states.prev = self._states.prev
    states.curr = self._states.curr
    states.next = self._states.next
    


  ##
  # @if jp
  # @brief 遷移の必要性チェック
  #
  # @param self
  #
  # @return 遷移必要性確認結果
  #
  # @else
  # @endif
  def need_trans(self):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    return (self._states.curr != self._states.next)


  ##
  # @if jp
  # @brief 現在状態の更新
  #
  # @param self
  # @param curr RTC.LifeCycleState
  #
  # @else
  # @endif
  def update_curr(self, curr):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    self._states.curr = curr
