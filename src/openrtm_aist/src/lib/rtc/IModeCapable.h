// -*- C++ -*-
/*!
 * @file  IModeCapable.h
 * @brief IModeCapable class
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

#ifndef RTC_LOCAL_IMODECAPABLE_H
#define RTC_LOCAL_IMODECAPABLE_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  class IMode;
  class IExecutionContext;
  /*!
   * @if jp
   * @class IModeCapable
   * @brief IModeCapable епеще╣
   * @else
   * @class IModeCapable
   * @brief IModeCapable class
   * @endif
   */
  class IModeCapable
  {
  public:
    virtual ~IModeCapable();
    virtual IMode&
    get_default_mode() = 0;
    virtual IMode&
    get_current_mode() = 0;
    virtual IMode&
    get_current_mode_in_context(const IExecutionContext& ec) = 0;
    virtual IMode&
    get_pending_mode() = 0;
    virtual IMode&
    get_pending_mode_in_context(const IExecutionContext& ec) = 0;
    virtual ReturnCode_t
    set_mode(IMode& new_mode, bool immediate) = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IMODECAPABLE_H

