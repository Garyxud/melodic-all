// -*- C++ -*-
/*!
 * @file  IFsmParticipantAction.h
 * @brief IFsmParticipantAction class
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

#ifndef RTC_LOCAL_IFSMPARTICIPANTACTION_H
#define RTC_LOCAL_IFSMPARTICIPANTACTION_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IFsmParticipantAction
   * @brief IFsmParticipantAction インターフェースクラス
   * @else
   * @class IFsmParticipantAction
   * @brief IFsmParticipantAction interface class
   * @endif
   */
  class IFsmParticipantAction
  {
  public:
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~IFsmParticipantAction() {};
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IFSMPARTICIPANTACTION_H

