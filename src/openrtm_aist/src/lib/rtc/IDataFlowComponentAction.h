// -*- C++ -*-
/*!
 * @file  IDataFlowComponentAction.h
 * @brief IDataFlowComponentAction interface class
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

#ifndef RTC_LOCAL_IDATAFLOWCOMPONENTACTION_H
#define RTC_LOCAL_IDATAFLOWCOMPONENTACTION_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IDataFlowComponentAction
   * @brief IDataFlowComponentAction インターフェースクラス
   * @else
   * @class IDataFlowComponentAction
   * @brief IDataFlowComponentAction class
   * @endif
   */
  class IDataFlowComponentAction
  {
  public:
    virtual ~IDataFlowComponentAction() {};
    virtual ReturnCode_t
    on_execute(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t
    on_state_update(ExecutionContextHandle_t ec_handle) = 0;
    virtual ReturnCode_t
    on_rate_changed(ExecutionContextHandle_t ec_handle) = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IDATAFLOWCOMPONENTACTION_H

