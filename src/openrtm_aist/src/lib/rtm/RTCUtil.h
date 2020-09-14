// -*- C++ -*-
/*!
 * @file RTCUtil.h
 * @brief RTComponent utils
 * @date $Date: 2007-12-31 03:08:06 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2007-2008
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef RTCUtil_h
#define RTCUtil_h

#include <rtm/idl/RTCSkel.h>
#include <rtm/idl/OpenRTMSkel.h>

/*!
 * @if jp
 * @namespace RTC_Utils
 *
 * @brief RTコンポーネント用ユーティリティ関数
 *
 * RTコンポーネントに対して以下のユーティリティ関数を提供する。
 * 
 * - isDataFlowParticipant
 * - isFsmParticipant
 * - isFsmObject
 * - isMultiModeObject
 *
 * @else
 * @namespace RTC_Utils
 *
 * @brief Utility functions for RT-Component
 *
 * This provides the following utility functions to RT-Component.
 * 
 * - isDataFlowParticipant
 * - isFsmParticipant
 * - isFsmObject
 * - isMultiModeObject
 *
 * @endif
 */
namespace RTC_Utils
{
  /*!
   * @if jp
   *
   * @brief DataFlowComponent であるか判定する
   *
   * 指定されたRTコンポーネントが DataFlowComponent であるか判定する。
   * DataFlowComponentは、 ExecutionContext の Semantics が
   * Periodic Sampled Data Processing の場合に利用されるRTコンポーネントの型
   * である。
   *
   * @param obj 判定対象の CORBA オブジェクト
   *
   * @return DataFlowComponent 判定結果
   *
   * @since 0.4.0
   *
   * @else
   *
   * @brief Confirm whether specified RT-Component is DataFlowComponent
   *
   * Confirm whether specified RT-Component is DataFlowComponent.
   * DataFlowComponent is a type of the RT-Component which is used 
   * when Semantics of ExecutionContext is Periodic Sampled Data Processing.
   *
   * @param obj The target CORBA object for the investigation
   *
   * @return Investigation result of DataFlowComponent
   *
   * @since 0.4.0
   *
   * @endif
   */
  bool isDataFlowComponent(CORBA::Object_ptr obj);
  
  /*!
   * @if jp
   *
   * @brief FsmParticipant であるか判定する
   *
   * 指定されたRTコンポーネントが FsmParticipant であるか判定する。
   * FsmParticipant は、 ExecutionContext の Semantics が
   * Stimulus Response Processing の場合に、状態内のアクションを定義するために
   * 利用されるRTコンポーネントの型である。
   *
   * @param obj 判定対象の CORBA オブジェクト
   *
   * @return FsmParticipant 判定結果
   *
   * @since 0.4.0
   *
   * @else
   *
   * @brief Confirm whether specified RT-Component is FsmParticipant
   *
   * Confirm whether specified RT-Component is FsmParticipant.
   * FsmParticipant is a type of the RT-Component which is used when Semantics
   * of ExecutionContext is Stimulus Response Processing. It is used to define 
   * the actions in the state.
   *
   * @param obj The target CORBA object for the investigation
   *
   * @return Investigation result of FsmParticipant
   *
   * @since 0.4.0
   *
   * @endif
   */
  bool isFsmParticipant(CORBA::Object_ptr obj);
  
  /*!
   * @if jp
   *
   * @brief Fsm であるか判定する
   *
   * 指定されたRTコンポーネントが Fsm であるか判定する。
   * Fsm は、 ExecutionContext の Semantics が Stimulus Response Processing の
   * 場合に、状態遷移を定義するために利用されるRTコンポーネントの型である。
   *
   * @param obj 判定対象の CORBA オブジェクト
   *
   * @return Fsm 判定結果
   *
   * @since 0.4.0
   *
   * @else
   *
   * @brief Confirm whether specified RT-Component is Fsm
   *
   * Confirm whether specified RT-Component is Fsm.
   * Fsm is a type of the RT-Component that is used when Semantics of 
   * ExecutionContext is Stimulus Response Processing. It is uset to define the 
   * state transition.
   *
   * @param obj The target CORBA object for the investigation
   *
   * @return Investigation result of Fsm
   *
   * @since 0.4.0
   *
   * @endif
   */
  bool isFsmObject(CORBA::Object_ptr obj);
  
  /*!
   * @if jp
   *
   * @brief multiModeComponent であるか判定する
   *
   * 指定されたRTコンポーネントが multiModeComponent であるか判定する。
   * multiModeComponent は、 ExecutionContext の Semantics が Modes of Operation 
   * の場合に、 Mode を定義するために利用されるRTコンポーネントの型である。
   *
   * @param obj 判定対象の CORBA オブジェクト
   *
   * @return multiModeComponent 判定結果
   *
   * @since 0.4.0
   *
   * @else
   *
   * @brief Confirm whether specified RT-Component is multiModeComponent
   *
   * Confirm whether specified RT-Component is multiModeComponent.
   * multiModeComponent is a type of the RT-Component which is used when
   * Semantics of ExecutionContext is Modes of Operation. It is used to define
   * Mode.
   *
   * @param obj The target CORBA object for the investigation
   *
   * @return Investigation result of multiModeComponent
   *
   * @since 0.4.0
   *
   * @endif
   */
  bool isMultiModeObject(CORBA::Object_ptr obj);
}; // namespace RTC_Utils
#endif // RTCUtil_h
