// -*- C++ -*-
/*!
 * @file  IFsmParticipant.h
 * @brief IFsmParticipant interface class
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

#ifndef RTC_LOCAL_IFSMPARTICIPANT_H
#define RTC_LOCAL_IFSMPARTICIPANT_H

#include <rtc/IRTObject.h>
#include <rtc/IFsmParticipantAction.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IFsmParticipant
   * @brief IFsmParticipant インターフェースクラス
   * @else
   * @class IFsmParticipant
   * @brief IFsmParticipant interface class
   * @endif
   */
  class IFsmParticipant
    : public virtual IRTObject,
      public virtual IFsmParticipantAction
  {
  public:
    virtual ~IFsmParticipant() {};
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IFSMPARTICIPANT_H

