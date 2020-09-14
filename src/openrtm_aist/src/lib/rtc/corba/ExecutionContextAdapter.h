// -*- C++ -*-
/*!
 * @file  ExecutionContextAdapter.h
 * @brief ExecutionContextAdapter
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
#ifndef RTC_CORBA_EXECUTIONCONTEXTADAPTER_H
#define RTC_CORBA_EXECUTIONCONTEXTADAPTER_H

#include <doil/ImplBase.h>
#include <rtc/IExecutionContext.h>
#include <rtc/ILightweightRTObject.h>
#include <rtc/corba/idl/RTCSkel.h>

namespace RTC
{
namespace CORBA
{
  typedef RTC::Local::ReturnCode_t ReturnCode_t;
  typedef RTC::Local::ExecutionKind ExecutionKind;
  typedef RTC::Local::LifeCycleState LifeCycleState;
  typedef RTC::Local::ExecutionContextProfile ExecutionContextProfile;
  typedef RTC::Local::ILightweightRTObject ILightweightRTObject;
  class ExecutionContextAdapter
    : public doil::ImplBase,
      public RTC::Local::IExecutionContext
  {
  public:
    ExecutionContextAdapter(RTC::ExecutionContext_ptr obj);

    virtual ~ExecutionContextAdapter();
    
    virtual bool is_running() const;
    
    virtual ReturnCode_t start();
    
    virtual ReturnCode_t stop();
    
    virtual double get_rate() const;
    
    virtual ReturnCode_t set_rate(double rate);
    
    virtual ReturnCode_t
    add_component(RTC::Local::ILightweightRTObject& comp);
    
    virtual ReturnCode_t
    remove_component(RTC::Local::ILightweightRTObject& comp);
    
    virtual ReturnCode_t
    activate_component(RTC::Local::ILightweightRTObject& comp);
    
    virtual ReturnCode_t
    deactivate_component(RTC::Local::ILightweightRTObject& comp);
    
    virtual ReturnCode_t
    reset_component(RTC::Local::ILightweightRTObject& comp);
    
    virtual LifeCycleState
    get_component_state(RTC::Local::ILightweightRTObject& comp);
    
    virtual ExecutionKind
    get_kind() const;
    
    virtual const ExecutionContextProfile&
    get_profile() const;
  private:
    RTC::ExecutionContext_ptr m_obj;
    ExecutionContextProfile m_profile;
  };

};     // namespace CORBA
};     // namespace RTC
#endif // RTC_CORBA_EXECUTIONCONTEXTADAPTER_H
