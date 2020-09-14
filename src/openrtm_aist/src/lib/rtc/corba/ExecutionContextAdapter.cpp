// -*- C++ -*-
/*!
 * @file  IRTC.h
 * @brief RTC interfaces
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
#include <doil/corba/util.h>
#include <rtc/corba/ExecutionContextAdapter.h>
#include <rtc/corba/util.h>

namespace RTC
{
namespace CORBA
{
  using namespace doil::CORBA;
  ExecutionContextAdapter::ExecutionContextAdapter(RTC::ExecutionContext_ptr obj)
    : m_obj(obj)
  {
  }

  ExecutionContextAdapter::~ExecutionContextAdapter()
  {
  }
  
  bool ExecutionContextAdapter::is_running() const
  {
    return m_obj->is_running();
  }
  
  ReturnCode_t ExecutionContextAdapter::start()
  {
    return to_local(m_obj->start());
  }
  
  ReturnCode_t ExecutionContextAdapter::stop()
  {
    return to_local(m_obj->stop());
  }
    
  double ExecutionContextAdapter::get_rate() const
  {
    return m_obj->get_rate();
  }
    
  ReturnCode_t ExecutionContextAdapter::set_rate(double rate)
  {
    return to_local(m_obj->set_rate(rate));
  }
  
  ReturnCode_t
  ExecutionContextAdapter::add_component(ILightweightRTObject& comp)
  {
    RTC::LightweightRTObject_ptr rtobj;
    rtobj = to_reference<RTC::LightweightRTObject>(&comp);
    return to_local(m_obj->add_component(rtobj));
  }
  
  ReturnCode_t
  ExecutionContextAdapter::remove_component(ILightweightRTObject& comp)
  {
    RTC::LightweightRTObject_ptr rtobj;
    rtobj = to_reference<RTC::LightweightRTObject>(&comp);
    return to_local(m_obj->remove_component(rtobj));
  }
  
  ReturnCode_t
  ExecutionContextAdapter::activate_component(ILightweightRTObject& comp)
  {
    RTC::LightweightRTObject_ptr rtobj;
    rtobj = to_reference<RTC::LightweightRTObject>(&comp);
    return to_local(m_obj->activate_component(rtobj));
  }
  
  ReturnCode_t
  ExecutionContextAdapter::deactivate_component(ILightweightRTObject& comp)
  {
    RTC::LightweightRTObject_ptr rtobj;
    rtobj = to_reference<RTC::LightweightRTObject>(&comp);
    return to_local(m_obj->deactivate_component(rtobj));
  }
  
  ReturnCode_t
  ExecutionContextAdapter::reset_component(ILightweightRTObject& comp)
  {
    RTC::LightweightRTObject_ptr rtobj;
    rtobj = to_reference<RTC::LightweightRTObject>(&comp);
    return to_local(m_obj->reset_component(rtobj));
  }
  
  LifeCycleState
  ExecutionContextAdapter::get_component_state(ILightweightRTObject& comp)
  {
    RTC::LightweightRTObject_ptr rtobj;
    rtobj = to_reference<RTC::LightweightRTObject>(&comp);
    return to_local(m_obj->get_component_state(rtobj));
  }
  
  ExecutionKind
  ExecutionContextAdapter::get_kind() const
  {
    return to_local(m_obj->get_kind());
  }
  
//  const ExecutionContextProfile&
//  ExecutionContextAdapter::get_profile() const
//  {
//    m_profile = to_local(m_obj->get_profile());
//    return m_profile;
//  }
};     // namespace CORBA
};     // namespace RTC
