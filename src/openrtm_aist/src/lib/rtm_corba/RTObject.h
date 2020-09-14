// -*- C++ -*-
/*!
 * @file RTObject.h
 * @brief RT component base class
 * @date $Date: 2008-01-14 07:57:18 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef RTObject_CORBA_h
#define RTObject_CORBA_h

// CORBA header include
#include "rtm/idl/RTCSkel.h"
#include "rtm/idl/OpenRTMSkel.h"

namespace RTC
{
  namespace CORBA
  {
    class RTObject_CORBA
      : public virtual POA_RTC::RTObject, 
        public virtual PortableServer::RefCountServantBase
    {
    public:
      RTObject_CORBA(RTC::Interface::RTObjectInterface* rtobj);
      virtual ~RTObject_impl();
      
      virtual ReturnCode_t initialize()
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t finalize()
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t exit()
        throw (CORBA::SystemException); 
      
      virtual CORBA::Boolean is_alive(ExecutionContext_ptr exec_context)
        throw (CORBA::SystemException);
      
      virtual ExecutionContext_ptr get_context(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ExecutionContextList* get_owned_contexts()
        throw (CORBA::SystemException);
      
      virtual ExecutionContextList* get_participating_contexts()
        throw (CORBA::SystemException);
      
      virtual ExecutionContextHandle_t
      get_context_handle(ExecutionContext_ptr cxt)
        throw (CORBA::SystemException);
      
      UniqueId attach_context(ExecutionContext_ptr exec_context)
        throw (CORBA::SystemException);
      
      ReturnCode_t detach_context(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ComponentProfile* get_component_profile()
        throw (CORBA::SystemException);
      
      virtual PortServiceList* get_ports()
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_initialize()
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_finalize()
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_startup(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_shutdown(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_activated(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_deactivated(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_aborting(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_error(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      virtual ReturnCode_t on_reset(UniqueId exec_handle)
        throw (CORBA::SystemException);
      
      
      //============================================================
      // SDOPackage::SdoSystemElement
      //============================================================
      virtual SDOPackage::OrganizationList* get_owned_organizations()
        throw (CORBA::SystemException, SDOPackage::NotAvailable);
      
      //============================================================
      // SDOPackage::SDO
      //============================================================
      virtual char* get_sdo_id()
        throw (CORBA::SystemException,
               SDOPackage::NotAvailable, SDOPackage::InternalError);
      
      virtual char* get_sdo_type()
        throw (CORBA::SystemException, 
               SDOPackage::NotAvailable, SDOPackage::InternalError);
      
      virtual SDOPackage::DeviceProfile* get_device_profile()
        throw (CORBA::SystemException, 
               SDOPackage::NotAvailable, SDOPackage::InternalError);
      
      virtual SDOPackage::ServiceProfileList* get_service_profiles()
        throw (CORBA::SystemException, 
               SDOPackage::NotAvailable, SDOPackage::InternalError);
      
      virtual SDOPackage::ServiceProfile* get_service_profile(const char* id)
        throw (CORBA::SystemException, 
               SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
               SDOPackage::InternalError);
      
      virtual SDOPackage::SDOService_ptr get_sdo_service(const char* id)
        throw (CORBA::SystemException, 
               SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
               SDOPackage::InternalError);
      
      virtual SDOPackage::Configuration_ptr get_configuration()
        throw (CORBA::SystemException, 
               SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable,
               SDOPackage::InternalError);
      
      virtual SDOPackage::Monitoring_ptr get_monitoring()
        throw (CORBA::SystemException, 
               SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable,
               SDOPackage::InternalError);
      
      virtual SDOPackage::OrganizationList* get_organizations()
        throw (CORBA::SystemException, 
               SDOPackage::NotAvailable, SDOPackage::InternalError);
      
      virtual SDOPackage::NVList* get_status_list()
        throw (CORBA::SystemException, 
               SDOPackage::NotAvailable, SDOPackage::InternalError);
      
      virtual CORBA::Any* get_status(const char* name)
        throw (CORBA::SystemException, 
               SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
               SDOPackage::InternalError);

    protected:
      class RTObjectInterface;
      std::auto_ptr<RTObjectInterface> m_rtobj;
      
    };
  };
};
#endif // RTObject
