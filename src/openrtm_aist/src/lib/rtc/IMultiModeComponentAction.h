// -*- C++ -*-
/*!
 * @file  IMultiiModeComponentAction.h
 * @brief IMultiiModeComponentAction class
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

#ifndef RTC_LOCAL_IMULTIIMODECOMPONENTACTION_H
#define RTC_LOCAL_IMULTIIMODECOMPONENTACTION_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IMultiiModeComponentAction
   * @brief IMultiiModeComponentAction クラス
   * @else
   * @class IMultiiModeComponentAction
   * @brief IMultiiModeComponentAction class
   * @endif
   */
  class IMultiiModeComponentAction
  {
  public:
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~IMultiiModeComponentAction() {};
    virtual ReturnCode_t on_mode_changed(ExecutionContextHandle_t ec_handle) = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IMULTIIMODECOMPONENTACTION_H

