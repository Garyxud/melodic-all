// -*- C++ -*-
/*!
 * @file RTObject.cpp
 * @brief RT component base class
 * @date $Date: 2008-01-14 07:57:15 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <assert.h>
#include "rtm/RTObject.h"

namespace RTC
{

  RTObject_impl::RTObject_impl(RTC::Interface::RTObjectInterface* rtobj)
    : m_rtobj(rtobj)
  {
  }
  
  RTObject_impl::~RTObject_impl()
  {
  }

  //============================================================
  // RTC::LightweightRTObject
  //============================================================
  ReturnCode_t RTObject_impl::initialize()
    throw (CORBA::SystemException)
  {
    return m_rtobj->initialize();
  }
  
  ReturnCode_t RTObject_impl::finalize()
    throw (CORBA::SystemException)
  {
    return m_rtobj->finalize();
  }

  ReturnCode_t RTObject_impl::exit()
    throw (CORBA::SystemException)
  {
    return m_rtobj->exit();
  }

  CORBA::Boolean
  RTObject_impl::is_alive(ExecutionContext_ptr exec_context)
    throw (CORBA::SystemException)
  {
    DObjectRegistry doreg;
    doreg = Manager::instance().DObjectRegistry();

    if (doreg.isLocal(exec_context))
      {
        ExecutionContextInterface& ec(doreg.getLocalObject(exec_context));
      }
    else
      {
        ExecutionContextInterface& ec(exec_context);
      }

    return m_rtobj->is_alive(ec);
  }
  
  /*!
   * @if jp
   * @brief [CORBA interface] ExecutionContextを取得する
   * @else
   * @brief [CORBA interface] Get ExecutionContext
   * @endif
   */
  ExecutionContext_ptr
  RTObject_impl::get_context(ExecutionContextHandle_t ec_id)
    throw (CORBA::SystemException)
  {
    ExecutionContextInterface& ecif(m_rtobj->get_context(ec_id));

    return Local2Corba(ecif);
  }
    
  /*!
   * @if jp
   * @brief [CORBA interface] ExecutionContextListを取得する
   * @else
   * @brief [CORBA interface] Get ExecutionContextList
   * @endif
   */
  ExecutionContextList* RTObject_impl::get_owned_contexts()
    throw (CORBA::SystemException)
  {
    ExecutionContextInterfaceList& eclist(m_rtobj->get_owned_contexts());

    ExecutionContextList_var retval;
    retval = new ExecutionContextList();
    CORBA_SeqUtil::for_each(eclist, ec_copy(retval));
    
    return retval._retn();
  }

  /*!
   * @if jp
   * @brief [CORBA interface] 参加している ExecutionContextList を取得する
   * @else
   * @brief [CORBA interface] Get participating ExecutionContextList.
   * @endif
   */
  ExecutionContextList* RTObject_impl::get_participating_contexts()
    throw (CORBA::SystemException)
  {
    ExecutionContextInterfaceList& eclist(m_rtobj->get_participating_contexts());
    ExecutionContextList_var retval;
    retval = new ExecutionContextList();
    CORBA_SeqUtil::for_each(eclist, ec_copy(retval));
    
    return retval._retn();
  }


  /*!
   * @if jp
   * @brief [CORBA interface] ExecutionContext のハンドルを返す
   * @else
   * @brief [CORBA interface] Return a handle of a ExecutionContext
   * @endif
   */
  ExecutionContextHandle_t
  RTObject_impl::get_context_handle(ExecutionContext_ptr cxt)
    throw (CORBA::SystemException)
  {
    ExecutionContextInterface ecif(Corba2Local(cxt));
    return m_rtobj->get_context_handle(ecif);
  }


  /*!
   * @if jp
   * @brief [CORBA interface] ExecutionContextをattachする
   * @else
   * @brief [CORBA interface] Attach ExecutionContext
   * @endif
   */
  UniqueId RTObject_impl::attach_context(ExecutionContext_ptr exec_context)
    throw (CORBA::SystemException)
  {
    return m_rtobj->attach_context(ExecutionContext_ptr exec_context);
  }
  
  /*!
   * @if jp
   * @brief [CORBA interface] ExecutionContextをdetachする
   * @else
   * @brief [CORBA interface] Detach ExecutionContext
   * @endif
   */
  ReturnCode_t RTObject_impl::detach_context(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->detach_context(UniqueId ec_id);
  }
  
  //============================================================
  // RTC::RTObject
  //============================================================
  
  /*!
   * @if jp
   * @brief [RTObject CORBA interface] コンポーネントプロファイルを取得する
   * @else
   * @brief [RTCObject CORBA interface] Get RTC's profile
   * @endif
   */
  ComponentProfile* RTObject_impl::get_component_profile()
    throw (CORBA::SystemException)
  {
    return m_rtobj->RTObject;
  }
  
  /*!
   * @if jp
   * @brief [RTObject CORBA interface] ポートを取得する
   * @else
   * @brief [RTCObject CORBA interface] Get Ports
   * @endif
   */
  PortServiceList* RTObject_impl::get_ports()
    throw (CORBA::SystemException)
  {
    return m_rtobj->get_ports();
  }
  
  //============================================================
  // RTC::ComponentAction
  //============================================================
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC の初期化
   * @else
   * @brief [ComponentAction CORBA interface] Initialize RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_initialize()
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_initialize();
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC の終了
   * @else
   * @brief [ComponentAction CORBA interface] Finalize RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_finalize()
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_finalize();
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC の開始
   * @else
   * @brief [ComponentAction CORBA interface] Startup RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_startup(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_startup(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC の停止
   * @else
   * @brief [ComponentAction CORBA interface] Shutdown RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_shutdown(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_shutdown(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC の活性化
   * @else
   * @brief [ComponentAction CORBA interface] Activate RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_activated(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_activated(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC の非活性化
   * @else
   * @brief [ComponentAction CORBA interface] Deactivate RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_deactivated(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_deactivated(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC のエラー状態への遷移
   * @else
   * @brief [ComponentAction interface] Transition to the error state
   * @endif
   */
  ReturnCode_t RTObject_impl::on_aborting(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_aborting(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC のエラー処理
   * @else
   * @brief [ComponentAction CORBA interface] Error Processing of RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_error(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_error(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [ComponentAction CORBA interface] RTC のリセット
   * @else
   * @brief [ComponentAction CORBA interface] Resetting RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_reset(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_reset(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [DataFlowComponentAction CORBA interface] RTC の定常処理(第一周期)
   * @else
   * @brief [DataFlowComponentAction CORBA interface] Primary Periodic 
   *        Operation of RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_execute(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_execute(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [DataFlowComponentAction CORBA interface] RTC の定常処理(第二周期)
   * @else
   * @brief [DataFlowComponentAction CORBA interface] Secondary Periodic 
   *        Operation of RTC
   * @endif
   */
  ReturnCode_t RTObject_impl::on_state_update(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_state_update(UniqueId ec_id);
  }
  
  /*!
   * @if jp
   * @brief [DataFlowComponentAction CORBA interface] 実行周期変更通知
   * @else
   * @brief [DataFlowComponentAction CORBA interface] Notify rate changed
   * @endif
   */
  ReturnCode_t RTObject_impl::on_rate_changed(UniqueId ec_id)
    throw (CORBA::SystemException)
  {
    return m_rtobj->on_rate_changed(UniqueId ec_id);
  }
  
  //============================================================
  // SDO interfaces
  //============================================================
  /*!
   * @if jp
   * @brief [SDO interface] Organization リストの取得 
   * @else
   * @brief [SDO interface] Get Organization list
   * @endif
   */
  SDOPackage::OrganizationList* RTObject_impl::get_owned_organizations()
    throw (CORBA::SystemException, SDOPackage::NotAvailable)
  {

  }
  
  // SDOPackage::SDO
  /*!
   * @if jp
   * @brief [SDO interface] SDO ID の取得
   * @else
   * @brief [SDO interface] Get the SDO ID
   * @endif
   */
  char* RTObject_impl::get_sdo_id()
    throw (CORBA::SystemException, 
	   SDOPackage::NotAvailable, SDOPackage::InternalError)
  {

  }
  
  /*!
   * @if jp
   * @brief [SDO interface] SDO タイプの取得
   * @else
   * @brief [SDO interface] Get SDO type
   * @endif
   */
  char* RTObject_impl::get_sdo_type()
    throw (CORBA::SystemException, 
	   SDOPackage::NotAvailable, SDOPackage::InternalError)
  {

  }
  
  /*!
   * @if jp
   * @brief [SDO interface] SDO DeviceProfile リストの取得 
   * @else
   * @brief [SDO interface] Get SDO DeviceProfile list
   * @endif
   */
  SDOPackage::DeviceProfile* RTObject_impl::get_device_profile()
    throw (CORBA::SystemException, 
	   SDOPackage::NotAvailable, SDOPackage::InternalError)
  {
    try
      {
	SDOPackage::DeviceProfile_var dprofile;
	dprofile = new SDOPackage::DeviceProfile();
	dprofile->device_type  = CORBA::string_dup(m_profile.category);
	dprofile->manufacturer = CORBA::string_dup(m_profile.vendor);
	dprofile->model        = CORBA::string_dup(m_profile.type_name);
	dprofile->version      = CORBA::string_dup(m_profile.version);
	dprofile->properties   = m_profile.properties;
	return dprofile._retn();
      }
    catch (...)
      {
	throw SDOPackage::InternalError("get_device_profile()");
      }
    return new SDOPackage::DeviceProfile();
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] SDO ServiceProfile の取得 
   * @else
   * @brief [SDO interface] Get SDO ServiceProfile
   * @endif
   */
  SDOPackage::ServiceProfileList* RTObject_impl::get_service_profiles()
    throw (CORBA::SystemException, 
	   SDOPackage::NotAvailable, SDOPackage::InternalError)
  {
    try
      {
	SDOPackage::ServiceProfileList_var sprofiles;
	sprofiles = new SDOPackage::ServiceProfileList(m_sdoSvcProfiles);
	return sprofiles._retn();
      }
    catch (...)
      {
	throw SDOPackage::InternalError("get_service_profiles()");
      }
    return new SDOPackage::ServiceProfileList();
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] 特定のServiceProfileの取得 
   * @else
   * @brief [SDO interface] Get specified ServiceProfile
   * @endif
   */
  SDOPackage::ServiceProfile*
  RTObject_impl::get_service_profile(const char* id)
    throw (CORBA::SystemException, 
	   SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
	   SDOPackage::InternalError)
  {
    if (!id)
      throw SDOPackage::InvalidParameter("get_service_profile(): Empty name.");
    
    try
      {
	CORBA::Long index;
	index = CORBA_SeqUtil::find(m_sdoSvcProfiles, svc_name(id));
	
	SDOPackage::ServiceProfile_var sprofile;
	sprofile = new SDOPackage::ServiceProfile(m_sdoSvcProfiles[index]);
	return sprofile._retn();
      }
    catch (...)
      {
	throw SDOPackage::InternalError("get_service_profile()");
      }
    return new SDOPackage::ServiceProfile();
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] 指定された SDO Service の取得
   * @else
   * @brief [SDO interface] Get specified SDO Service's reference
   * @endif
   */
  SDOPackage::SDOService_ptr RTObject_impl::get_sdo_service(const char* id)
    throw (CORBA::SystemException, 
	   SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
	   SDOPackage::InternalError)
  {
    if (!id)
      throw SDOPackage::InvalidParameter("get_service(): Empty name.");
    
    try
      {
	CORBA::Long index;
	index = CORBA_SeqUtil::find(m_sdoSvcProfiles, svc_name(id));
	
	SDOPackage::SDOService_var service;
	service = m_sdoSvcProfiles[index].service;
	return service._retn();
      }
    catch (...)
      {
	throw SDOPackage::InternalError("get_service()");
      }
    return SDOPackage::SDOService::_nil();
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] Configuration オブジェクトの取得 
   * @else
   * @brief [SDO interface] Get Configuration object
   * @endif
   */
  SDOPackage::Configuration_ptr RTObject_impl::get_configuration()
    throw (CORBA::SystemException, 
	   SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable,
	   SDOPackage::InternalError)
  {
    if (m_pSdoConfig == NULL)
      throw SDOPackage::InterfaceNotImplemented();
    try
      {
	SDOPackage::Configuration_var config;
	config = m_pSdoConfig;
	return config._retn();
      }
    catch (...)
      {
	SDOPackage::InternalError("get_configuration()");
      }
    return SDOPackage::Configuration::_nil();
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] Monitoring オブジェクトの取得 
   * @else
   * @brief [SDO interface] Get Monitoring object
   * @endif
   */
  SDOPackage::Monitoring_ptr RTObject_impl::get_monitoring()
    throw (CORBA::SystemException, 
	   SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable,
	   SDOPackage::InternalError)
  {
    throw SDOPackage::InterfaceNotImplemented();
    return SDOPackage::Monitoring::_nil();
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] Organization リストの取得 
   * @else
   * @brief [SDO interface] Get Organization list
   * @endif
   */
  SDOPackage::OrganizationList* RTObject_impl::get_organizations()
    throw (CORBA::SystemException, 
	   SDOPackage::NotAvailable, SDOPackage::InternalError)
  {
    try
      {
	SDOPackage::OrganizationList_var org;
	org = new SDOPackage::OrganizationList(m_sdoOrganizations);
	return org._retn();
      }
    catch (...)
      {
	throw SDOPackage::InternalError("get_organizations()");
      }
    return new SDOPackage::OrganizationList(0);
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] SDO Status リストの取得 
   * @else
   * @brief [SDO interface] Get SDO Status list
   * @endif
   */
  SDOPackage::NVList* RTObject_impl::get_status_list()
    throw (CORBA::SystemException, 
	   SDOPackage::NotAvailable, SDOPackage::InternalError)
  {
    try
      {
	NVList_var status;
	status = new NVList(m_sdoStatus);
	return status._retn();
      }
    catch (...)
      {
	SDOPackage::InternalError("get_status_list()");
      }
    return new SDOPackage::NVList(0);
  }
  
  /*!
   * @if jp
   * @brief [SDO interface] SDO Status の取得 
   * @else
   * @brief [SDO interface] Get SDO Status
   * @endif
   */
  CORBA::Any* RTObject_impl::get_status(const char* name)
    throw (CORBA::SystemException, 
	   SDOPackage::InvalidParameter, SDOPackage::NotAvailable,
	   SDOPackage::InternalError)
  {
    CORBA::Long index;
    index = CORBA_SeqUtil::find(m_sdoStatus, nv_name(name));
    if (index < 0)
      throw SDOPackage::InvalidParameter("get_status(): Not found");
    try
      {
	CORBA::Any_var status;
	status = new CORBA::Any(m_sdoStatus[index].value);
	return status._retn();
      }
    catch (...)
      {
	throw SDOPackage::InternalError("get_status()");
      }
    return new CORBA::Any();
  }
  
  //============================================================
  // Local methods
  //============================================================
  /*!
   * @if jp
   * @brief [local interface] インスタンス名の設定
   * @else
   * @brief [local interface] Set instance name
   * @endif
   */
  void RTObject_impl::setInstanceName(const char* instance_name)
  {
    m_properties["instance_name"] = instance_name;
    m_profile.instance_name = m_properties["instance_name"].c_str();
  }
  
  /*!
   * @if jp
   * @brief [local interface] Naming Server 情報の取得
   * @else
   * @brief [local interface] Get Naming Server information
   * @endif
   */
  std::vector<std::string> RTObject_impl::getNamingNames()
  {
    return split(m_properties["naming.names"], ",");
  }
  
  /*!
   * @if jp
   * @brief [local interface] オブジェクトリファレンスの設定
   * @else
   * @brief [local interface] Set the object reference
   * @endif
   */
  void RTObject_impl::setObjRef(const RTObject_ptr rtobj)
  {
    m_objref = rtobj;
  }
  
  /*!
   * @if jp
   * @brief [local interface] オブジェクトリファレンスの取得
   * @else
   * @brief [local interface] Get the object reference
   * @endif
   */
  RTObject_ptr RTObject_impl::getObjRef() const
  {
    return RTC::RTObject::_duplicate(m_objref);
  }
  
  /*!
   * @if jp
   * @brief [local interface] RTC のプロパティを設定する
   * @else
   * @brief [local interface] Set RTC property
   * @endif
   */
  void RTObject_impl::setProperties(const Properties& prop)
  {
    m_properties << prop;
    m_profile.instance_name = m_properties["instance_name"].c_str();
    m_profile.type_name     = m_properties["type_name"].c_str();
    m_profile.description   = m_properties["description"].c_str();
    m_profile.version       = m_properties["version"].c_str();
    m_profile.vendor        = m_properties["vendor"].c_str();
    m_profile.category      = m_properties["category"].c_str();
  }
  
  /*!
   * @if jp
   * @brief [local interface] RTC のプロパティを取得する
   * @else
   * @brief [local interface] Get RTC property
   * @endif
   */
  Properties& RTObject_impl::getProperties()
  {
    return m_properties;
  }
  
  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの更新(ID指定)
   * @else
   * @brief Update configuration parameters (by ID)
   * @endif
   */
  void RTObject_impl::updateParameters(const char* config_set)
  {
    m_configsets.update(config_set);
    return;
  }
  
  /*!
   * @if jp
   * @brief [local interface] Port を登録する
   * @else
   * @brief [local interface] Register Port
   * @endif
   */
  void RTObject_impl::registerPort(PortBase& port)
  {
    m_portAdmin.registerPort(port);
    port.setOwner(this->getObjRef());
    return;
  }
  
  /*!
   * @if jp
   * @brief [local interface] Port の登録を削除する
   * @else
   * @brief [local interface] Unregister Port
   * @endif
   */
  void RTObject_impl::deletePort(PortBase& port)
  {
    m_portAdmin.deletePort(port);
    return;
  }
  
  /*!
   * @if jp
   * @brief [local interface] 名前指定により Port の登録を削除する
   * @else
   * @brief [local interface] Delete Port by specifying its name
   * @endif
   */
  void RTObject_impl::deletePortByName(const char* port_name)
  {
    m_portAdmin.deletePortByName(port_name);
    return;
  }
  
  /*!
   * @if jp
   * @brief 全 Port の登録を削除する
   * @else
   * @brief Unregister All Ports
   * @endif
   */
  void RTObject_impl::finalizePorts()
  {
    m_portAdmin.finalizePorts();
  }
  
  /*!
   * @if jp
   * @brief RTC を終了する
   * @else
   * @brief Shutdown RTC
   * @endif
   */
  void RTObject_impl::shutdown()
  {
    try
      {
	finalizePorts();
	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(m_pSdoConfigImpl));
	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(this));
      }
    catch (...)
      {
	;
      }
    
    if (m_pManager != NULL)
      {
	m_pManager->cleanupComponent(this);
      }
  }
};
