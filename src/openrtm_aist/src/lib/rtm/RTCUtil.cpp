// -*- C++ -*-
/*!
 * @file RTCUtil.h
 * @brief RTComponent utils
 * @date $Date: 2007-12-31 03:08:06 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2007-2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <rtm/RTC.h>
#include <rtm/RTCUtil.h>

namespace RTC_Utils
{
  /*!
   * @if jp
   * @brief DataFlowComponent であるか判定する
   * @else
   * @brief Confirm whether specified RT-Component is DataFlowComponent
   * @endif
   */
  bool isDataFlowComponent(CORBA::Object_ptr obj)
  {
    OpenRTM::DataFlowComponent_var dfp;
    dfp = OpenRTM::DataFlowComponent::_narrow(obj);
    return !CORBA::is_nil(dfp);
  }
  
  /*!
   * @if jp
   * @brief FsmParticipant であるか判定する
   * @else
   * @brief Confirm whether specified RT-Component is FsmParticipant
   * @endif
   */
  bool isFsmParticipant(CORBA::Object_ptr obj)
  {
    RTC::FsmParticipant_var fsmp;
    fsmp = RTC::FsmParticipant::_narrow(obj);
    return !CORBA::is_nil(fsmp);
  }
  
  /*!
   * @if jp
   * @brief Fsm であるか判定する
   * @else
   * @brief Confirm whether specified RT-Component is Fsm
   * @endif
   */
  bool isFsmObject(CORBA::Object_ptr obj)
  {
    RTC::FsmObject_var fsm;
    fsm = RTC::FsmObject::_narrow(obj);
    return !CORBA::is_nil(fsm);
  }
  
  /*!
   * @if jp
   * @brief multiModeComponent であるか判定する
   * @else
   * @brief Confirm whether specified RT-Component is multiModeComponent
   * @endif
   */
  bool isMultiModeObject(CORBA::Object_ptr obj)
  {
    RTC::MultiModeObject_var mmc;
    mmc = RTC::MultiModeObject::_narrow(obj);
    return !CORBA::is_nil(mmc);
  }
}; // namespace RTC_Utils

