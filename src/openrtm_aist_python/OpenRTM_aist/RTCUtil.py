#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# @file RTCUtil.py
# @brief RTComponent utils
# @date $Date: 2007/09/11 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2007-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

from omniORB import CORBA

import RTC
import OpenRTM

##
# @if jp
#
# @brief DataFlowComponent であるか判定する
#
# 指定されたRTコンポーネントが DataFlowComponent であるか判定する。
# DataFlowComponent、 ExecutionContext の Semantics が
# Periodic Sampled Data Processing の場合に利用されるRTコンポーネントの型
# である。
#
# @param obj 判定対象の CORBA オブジェクト
#
# @return DataFlowComponent 判定結果
#
# @since 0.4.0
#
# @else
#
# @endif
def isDataFlowComponent(obj):
  dfp = obj._narrow(OpenRTM.DataFlowComponent)
  return not CORBA.is_nil(dfp)


##
# @if jp
#
# @brief FsmParticipant であるか判定する
#
# 指定されたRTコンポーネントが FsmParticipant であるか判定する。
# FsmParticipant は、 ExecutionContext の Semantics が
# Stimulus Response Processing の場合に、状態内のアクションを定義するために
# 利用されるRTコンポーネントの型である。
#
# @param obj 判定対象の CORBA オブジェクト
#
# @return FsmParticipant 判定結果
#
# @since 0.4.0
#
# @else
#
# @endif
def isFsmParticipant(obj):
  fsmp = obj._narrow(RTC.FsmParticipant)
  return not CORBA.is_nil(fsmp)


##
# @if jp
#
# @brief Fsm であるか判定する
#
# 指定されたRTコンポーネントが Fsm であるか判定する。
# Fsm は、 ExecutionContext の Semantics が Stimulus Response Processing の
# 場合に、状態遷移を定義するために利用されるRTコンポーネントの型である。
#
# @param obj 判定対象の CORBA オブジェクト
#
# @return Fsm 判定結果
#
# @since 0.4.0
#
# @else
#
# @endif
def isFsmObject(obj):
  fsm = obj._narrow(RTC.FsmObject)
  return not CORBA.is_nil(fsm)


##
# @if jp
#
# @brief multiModeComponent であるか判定する
#
# 指定されたRTコンポーネントが multiModeComponent であるか判定する。
# multiModeComponent は、 ExecutionContext の Semantics が Modes of Operatin 
# の場合に、 Mode を定義するために利用されるRTコンポーネントの型である。
#
# @param obj 判定対象の CORBA オブジェクト
#
# @return multiModeComponent 判定結果
#
# @since 0.4.0
#
# @else
#
# @endif
def isMultiModeObject(obj):
  mmc = obj._narrow(RTC.MultiModeObject)
  return not CORBA.is_nil(mmc)
