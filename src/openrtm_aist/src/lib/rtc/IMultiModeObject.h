// -*- C++ -*-
/*!
 * @file  IMultiModeObject.h
 * @brief IMultiModeObject class
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

#ifndef RTC_LOCAL_IMULTIMODEOBJECT_H
#define RTC_LOCAL_IMULTIMODEOBJECT_H

#include <rtc/IRTC.h>
#include <rtc/IModeCapable.h>
#include <rtc/IMultModeComponentAction.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IMultiModeObject
   * @brief IMultiModeObject クラス
   * @else
   * @class IMultiModeObject
   * @brief IMultiModeObject class
   * @endif
   */
  class IMultiModeObject
    : public virtual IRTObject,
      public virtual IModeCapable,
      public virtual IMultiModeComponentAction
  {
  public:
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~IMultiModeObject() {};
    virtual ReturnCode_t
    on_mode_changed(ExecutionContextHandle_t ec_handle) = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IMULTIMODEOBJECT_H

