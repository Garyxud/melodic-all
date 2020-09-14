// -*- C++ -*-
/*!
 * @file  IDataFlowComponent.h
 * @brief IDataFlowComponent interface class
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

#ifndef RTC_LOCAL_IDATAFLOWCOMPONENT_H
#define RTC_LOCAL_IDATAFLOWCOMPONENT_H

#include <rtc/IRTC.h>
#include <rtc/IRTObject.h>
#include <rtc/IDataFlowComponentAction.h> 

namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class IDataFlowComponent
   * @brief IDataFlowComponent インターフェースクラス
   * @else
   * @class IDataFlowComponent
   * @brief IDataFlowComponent class
   * @endif
   */
  class IDataFlowComponent
    : public virtual IRTObject,
      public virtual IDataFlowComponentAction
  {
  public:
    virtual ~IDataFlowComponent() {};
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IDATAFLOWCOMPONENT_H

