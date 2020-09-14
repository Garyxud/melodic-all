// -*- C++ -*-
/*!
 * @file  IFsmService.h
 * @brief IFsmService class
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
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

#ifndef RTC_LOCAL_IFSMSERVICE_H
#define RTC_LOCAL_IFSMSERVICE_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  class IFsmParticipantAction;
  
  struct FsmBehaviorProfile
  {
    IFsmParticipantAction* comp;
    UniqueIdentifier id;
  };
  
  typedef std::vector<FsmBehaviorProfile*> FsmBehaviorProfileList;
  
  struct FsmProfile
  {
    FsmBehaviorProfileList behavior_profiles;
  };
  
  /*!
   * @if jp
   * @class IFsmService
   * @brief IFsmService епеще╣
   * @else
   * @class IFsmService
   * @brief IFsmService class
   * @endif
   */
  class IFsmService
  {
  public:
    virtual ~IFsmService() {};
    virtual const FsmProfile& get_fsm_profile() = 0;
    virtual ReturnCode_t set_fsm_profile(const FsmProfile& fsm_profile) = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IFSMSERVICE_H

