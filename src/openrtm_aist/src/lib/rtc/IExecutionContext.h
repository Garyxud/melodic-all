// -*- C++ -*-
/*!
 * @file  IExecutionContext.h
 * @brief IExecutionContext interface class
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

#ifndef IExeLOCAL_cutionContext_h
#define IExeLOCAL_cutionContext_h

#include <rtc/IRTC.h>


namespace RTC
{
namespace Local
{

  class ILightweightRTObject;
  class IRTObject;
  typedef std::vector<IRTObject*> RTCList;
  
  struct ExecutionContextProfile
  {
    ExecutionKind kind;
    double rate;
    IRTObject* owner;
    RTCList participants;
    NVList properties;
  };
  
  /*!
   * @if jp
   * @class IExecutionContext
   * @brief IExecutionContext епеще╣
   * @else
   * @class IExecutionContext
   * @brief IExecutionContext class
   * @endif
   */
  class IExecutionContext
  {
  public:
    virtual ~IExecutionContext() {};
    
    virtual bool is_running() const = 0;
    
    virtual ReturnCode_t start() = 0;
    
    virtual ReturnCode_t stop() = 0;
    
    virtual double get_rate() const = 0;
    
    virtual ReturnCode_t set_rate(double rate) = 0;
    
    virtual ReturnCode_t
    add_component(ILightweightRTObject& comp) = 0;
    
    virtual ReturnCode_t
    remove_component(ILightweightRTObject& comp) = 0;
    
    virtual ReturnCode_t
    activate_component(ILightweightRTObject& comp) = 0;
    
    virtual ReturnCode_t
    deactivate_component(ILightweightRTObject& comp) = 0;
    
    virtual ReturnCode_t
    reset_component(ILightweightRTObject& comp) = 0;
    
    virtual LifeCycleState
    get_component_state(ILightweightRTObject& comp) = 0;
    
    virtual ExecutionKind
    get_kind() const = 0;
    
    virtual const ExecutionContextProfile&
    get_profile() const = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IEXECUTIONCONTEXT_H

