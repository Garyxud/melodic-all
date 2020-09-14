// -*- C++ -*-
/*!
 * @file  ILightweightRTObject.h
 * @brief ILightweightRTObject interface class
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

#ifndef RTC_LOCAL_ILIGHTWEIGHTRTOBJECT_H
#define RTC_LOCAL_ILIGHTWEIGHTRTOBJECT_H

#include <doil/ImplBase.h>
#include <rtc/IRTC.h>
#include <rtc/IComponentAction.h>
#include <rtc/IExecutionContext.h>

namespace RTC
{
namespace Local
{
    /*!
     * @if jp
     * @class ILightweightRTObject
     * @brief ILightweightRTObject インターフェースクラス
     * @else
     * @class ILightweightRTObject
     * @brief ILightweightRTObject interface class
     * @endif
     */
    class ILightweightRTObject
      : public doil::ImplBase,
        public virtual IComponentAction
    {
    public:
      virtual ~ILightweightRTObject() {};
      virtual bool is_alive(const IExecutionContext& ec) const = 0;
      virtual ReturnCode_t initialize() = 0;
      virtual ReturnCode_t finalize() = 0;
      virtual ReturnCode_t exit() = 0;

      virtual ExecutionContextHandle_t
      attach_context(const IExecutionContext& ec) = 0;
      virtual ReturnCode_t detach_context(ExecutionContextHandle_t ec_handle) = 0;
      virtual IExecutionContext&
      get_context(ExecutionContextHandle_t ec_handle) = 0;
      virtual ExecutionContextList& get_owned_contexts() = 0;
      virtual ExecutionContextList& get_participating_contexts() = 0;
    };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_ILIGHTWEIGHTRTOBJECT_H
