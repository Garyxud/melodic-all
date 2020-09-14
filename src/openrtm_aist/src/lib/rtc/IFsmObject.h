// -*- C++ -*-
/*!
 * @file  IFsmObject.h
 * @brief IFsmObject interface class
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

#ifndef RTC_LOCAL_IFSMOBJECT_H
#define RTC_LOCAL_IFSMOBJECT_H

#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
    /*!
     * @if jp
     * @class IFsmObject
     * @brief IFsmObject インターフェースクラス
     * @else
     * @class IFsmObject
     * @brief IFsmObject interface class
     * @endif
     */
    class IFsmObject
    {
    public:
      virtual ~IFsmObject() {};
      virtual ReturnCode_t
      send_stimulus(const char* message,
		    ExecutionContextHandle_t exec_context) = 0;
    };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IFSMOBJECT_H
  
  
