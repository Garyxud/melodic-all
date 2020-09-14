// -*- C++ -*-
/*!
 * @file  IComponentAction.h
 * @brief IComponentAction interface class
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

#ifndef RTC_LOCAL_ICOMPONENTACTION_H
#define RTC_LOCAL_ICOMPONENTACTION_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IComponentAction
   * @brief IComponentAction インターフェースクラス
   * @else
   * @class IComponentAction
   * @brief IComponentAction interface class
   * @endif
   */
  class IComponentAction
  {
  public:
    virtual ~IComponentAction() {};
    
    virtual ReturnCode_t on_initialize() = 0;
    virtual ReturnCode_t on_finalize() = 0;
    
    virtual ReturnCode_t on_startup(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t on_shutdown(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t on_activated(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t on_deactivated(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t on_aborting(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t on_error(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t on_reset(ExecutionContextHandle_t ec_handle) = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_ICOMPONENTACTION_H

