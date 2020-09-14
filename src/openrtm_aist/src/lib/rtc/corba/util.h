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

#ifndef RTC_CORBA_UTIL_H
#define RTC_CORBA_UTIL_H

#include <rtc/IRTC.h>
#include <rtc/corba/idl/RTCSkel.h>
#include <doil/ImplBase.h>
#include <assert.h>


namespace RTC
{
namespace CORBA
{
  template <typename Local, typename Remote>
  bool to_remote(Local* lobj, Remote* robj)
  {
    ::doil::ImplBase* l;
    l = dynamic_cast<doil::ImplBase*>(lobj);
    if (l != NULL) return false;
    ::CORBA::Object_ptr r;
    r = doil::CORBA::CORBAManager::instance().toReference(l);
    if (!::CORBA::is_nil(r)) return false;
    robj = Remote::_narrow(r);
    return true;
  }

  /*!
   * @brief RTC::ReturnCode_t -> RTC::Local::ReturnCode_t conversion
   */
  inline RTC::Local::ReturnCode_t to_local(RTC::ReturnCode_t x)
  {
    return RTC::Local::ReturnCode_t((int)x);
  }

  /*!
   * @brief RTC::Local::ReturnCode_t -> RTC::ReturnCode_t conversion
   */
  inline RTC::ReturnCode_t to_remote(RTC::Local::ReturnCode_t x)
  {
    return RTC::ReturnCode_t((int)x);
  }

  /*!
   * @brief RTC::ExecutionKind -> RTC::Local::ExecutionKind
   */
  inline RTC::Local::ExecutionKind to_local(RTC::ExecutionKind x)
  {
    return RTC::Local::ExecutionKind((int)x);
  }

  /*!
   * @brief RTC::Local::ExecutionKind -> RTC::ExecutionKind conversion
   */
  inline RTC::ExecutionKind to_remote(RTC::Local::ExecutionKind x)
  {
    return RTC::ExecutionKind((int)x);
  }
  
  /*!
   * @brief RTC::PortInterfacePolarity -> RTC::Local::PortInterfacePolarity
   */
  inline RTC::Local::PortInterfacePolarity
  to_local(RTC::PortInterfacePolarity x)
  {
    return RTC::Local::PortInterfacePolarity((int)x);
  }

  /*!
   * @brief RTC::Local::PortInterfacePolarity -> RTC::PortInterfacePolarity
   */
  inline RTC::PortInterfacePolarity to_remote(RTC::Local::PortInterfacePolarity x)
  {
    return RTC::PortInterfacePolarity((int)x);
  }

  /*!
   * @brief RTC::LifeCycleState -> RTC::Local::LifeCycleState
   */
  inline RTC::Local::LifeCycleState
  to_local(RTC::LifeCycleState x)
  {
    return RTC::Local::LifeCycleState((int)x);
  }

  /*!
   * @brief RTC::Local::LifeCycleState -> RTC::LifeCycleState
   */
  inline RTC::LifeCycleState to_remote(RTC::Local::LifeCycleState x)
  {
    return RTC::LifeCycleState((int)x);
  }



  inline RTC::Local::ExecutionContextProfile to_local(RTC::ExecutionContextProfile x)
  {
    RTC::Local::ExecutionContextProfile l;
    l.kind = to_local(x.kind);
    l.rate = x.rate;
    //    l.owner = NULL;
    //    l.participoants = 
    // properties =
    return l;
  }

};
};
#endif // RTC_CORBA_UTIL_H
