// -*- C++ -*-
/*!
 * @file RTObjectServant.h
 * @brief RTObject corba servant class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef RTC_RTOBJECTSERVANT_H
#define RTC_RTOBJECTSERVANT_H

#include <memory>

// doil include
#include <doil/ImplBase.h>
#include <doil/corba/CORBA.h>
#include <doil/corba/CORBAServantBase.h>
#include <doil/corba/CORBAManager.h>

// rtc include
#include <rtc/IRTC.h>

// CORBA header include
#include "rtc/corba/idl/RTCSkel.h"
#include "rtc/corba/idl/OpenRTMSkel.h"



namespace RTC
{
namespace Local
{
  class IRTObject;
};
namespace CORBA
{
  typedef ::RTC::Local::IRTObject IRTObject;
  typedef RTC::ExecutionContextHandle_t EChandle;
  
  RTC::ReturnCode_t ret(RTC::Local::ReturnCode_t r)
  {
    return RTC::ReturnCode_t((int)r);
  }
  
  class RTObjectServant
    : public virtual POA_RTC::RTObject, 
      public virtual doil::CORBA::CORBAServantBase
  {
  public:
    RTObjectServant(doil::ImplBase* impl);
    virtual ~RTObjectServant();
    
    //============================================================
    // RTC::LightweightRTObject
    //============================================================
    virtual ReturnCode_t initialize()
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t finalize()
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t exit()
      throw (::CORBA::SystemException); 
    
    virtual ::CORBA::Boolean is_alive(ExecutionContext_ptr exec_context)
      throw (::CORBA::SystemException);
    
    virtual ExecutionContext_ptr get_context(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ExecutionContextList* get_owned_contexts()
      throw (::CORBA::SystemException);
    
    virtual ExecutionContextList* get_participating_contexts()
      throw (::CORBA::SystemException);
    
    virtual ExecutionContextHandle_t
    get_context_handle(ExecutionContext_ptr cxt)
      throw (::CORBA::SystemException);
    
    EChandle attach_context(ExecutionContext_ptr exec_context)
      throw (::CORBA::SystemException);
    
    ReturnCode_t detach_context(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ComponentProfile* get_component_profile()
      throw (::CORBA::SystemException);
    
    virtual PortServiceList* get_ports()
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_initialize()
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_finalize()
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_startup(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_shutdown(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_activated(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_deactivated(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_aborting(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_error(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    virtual ReturnCode_t on_reset(EChandle exec_handle)
      throw (::CORBA::SystemException);
    
    
    //============================================================
    // SDOPackage::SdoSystemElement
    //============================================================
    virtual SDOPackage::OrganizationList* get_owned_organizations()
      throw (::CORBA::SystemException, SDOPackage::NotAvailable);
    
    //============================================================
    // SDOPackage::SDO
    //============================================================
    virtual char* get_sdo_id()
      throw (::CORBA::SystemException,
             SDOPackage::NotAvailable, SDOPackage::InternalError);
    
    virtual char* get_sdo_type()
      throw (::CORBA::SystemException, 
             SDOPackage::NotAvailable, SDOPackage::InternalError);
    
    virtual SDOPackage::DeviceProfile* get_device_profile()
      throw (::CORBA::SystemException, 
             SDOPackage::NotAvailable, SDOPackage::InternalError);
    
    virtual SDOPackage::ServiceProfileList* get_service_profiles()
      throw (::CORBA::SystemException, 
             SDOPackage::NotAvailable, SDOPackage::InternalError);
    
    virtual SDOPackage::ServiceProfile* get_service_profile(const char* id)
      throw (::CORBA::SystemException, 
             SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
             SDOPackage::InternalError);
    
    virtual SDOPackage::SDOService_ptr get_sdo_service(const char* id)
      throw (::CORBA::SystemException, 
             SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
             SDOPackage::InternalError);
    
    virtual SDOPackage::Configuration_ptr get_configuration()
      throw (::CORBA::SystemException, 
             SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable,
             SDOPackage::InternalError);
    
    virtual SDOPackage::Monitoring_ptr get_monitoring()
      throw (::CORBA::SystemException, 
             SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable,
             SDOPackage::InternalError);
    
    virtual SDOPackage::OrganizationList* get_organizations()
      throw (::CORBA::SystemException, 
             SDOPackage::NotAvailable, SDOPackage::InternalError);
    
    virtual SDOPackage::NVList* get_status_list()
      throw (::CORBA::SystemException, 
             SDOPackage::NotAvailable, SDOPackage::InternalError);
    
    virtual ::CORBA::Any* get_status(const char* name)
      throw (::CORBA::SystemException, 
             SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
             SDOPackage::InternalError);
    
  protected:
    ::RTC::Local::IRTObject* m_impl;
    
  };
};
};
#endif // RTObject
