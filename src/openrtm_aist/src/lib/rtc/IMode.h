// -*- C++ -*-
/*!
 * @file  IMode.h
 * @brief IMode interface class
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

#ifndef RTC_LOCAL_IMODE_H
#define RTC_LOCAL_IMODE_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IMode
   * @brief IMode епеще╣
   * @else
   * @class IMode
   * @brief IMode class
   * @endif
   */
  class IMode
  {
  public:
    virtual ~IMode() {};
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IMODE_H

