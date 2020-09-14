// -*- C++ -*-
/*!
 * @file SdoServiceAdmin.cpp
 * @brief SDO service administration class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2011
 *     Noriaki Ando
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: SdoConfiguration.cpp 1971 2010-06-03 08:46:40Z n-ando $
 *
 */

#include <memory>
#include <coil/UUID.h>
#include <coil/Guard.h>
#include <coil/stringutil.h>
#include <rtm/RTObject.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/SdoServiceAdmin.h>
#include <rtm/SdoServiceProviderBase.h>
#include <rtm/SdoServiceConsumerBase.h>

namespace RTC
{
  typedef coil::Guard<coil::Mutex> Guard;

  /*!
   * @if jp
   * @brief  ServiceProfile用functor
   * @else
   * @brief  Functor for ServiceProfile
   * @endif
   */
  struct service_id
  {
    service_id(const char* id) : m_id(id) {};
    bool operator()(const SDOPackage::ServiceProfile& s)
    {
      std::string id(s.id);
      return m_id == id;
    }
    const std::string m_id;
  };
  

  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  SdoServiceAdmin::SdoServiceAdmin(::RTC::RTObject_impl& rtobj)
    : m_rtobj(rtobj), m_allConsumerEnabled(true),
      rtclog("SdoServiceAdmin")
  {
    RTC_TRACE(("SdoServiceAdmin::SdoServiceAdmin(%s)",
               rtobj.getProperties()["instance_name"].c_str()));

    ::coil::Properties& prop(m_rtobj.getProperties());

    //------------------------------------------------------------
    // SDO service provider
   ::coil::vstring enabledProviderTypes 
      = ::coil::split(prop["sdo.service.provider.enabled_services"], ",", true);
    RTC_DEBUG(("sdo.service.provider.enabled_services: %s",
               prop["sdo.service.provider.enabled_services"].c_str()));

    ::coil::vstring availableProviderTypes 
      = SdoServiceProviderFactory::instance().getIdentifiers();
    prop["sdo.service.provider.available_services"]
      = coil::flatten(availableProviderTypes);
    RTC_DEBUG(("sdo.service.provider.available_services: %s",
               prop["sdo.service.provider.available_services"].c_str()));

    
    // If types include '[Aa][Ll][Ll]', all types enabled in this RTC
    ::coil::vstring activeProviderTypes;
    for (size_t i(0); i < enabledProviderTypes.size(); ++i)
      {
        std::string tmp(enabledProviderTypes[i]);
        coil::toLower(tmp);
        if (tmp == "all")
          {
            activeProviderTypes = availableProviderTypes;
            RTC_DEBUG(("sdo.service.provider.enabled_services: ALL"));
            break;
          }
        for (size_t j(0); j < availableProviderTypes.size(); ++j)
          {
            if (availableProviderTypes[j] == enabledProviderTypes[i])
              {
                activeProviderTypes.push_back(availableProviderTypes[j]);
              }
          }
      }

    SdoServiceProviderFactory& factory(SdoServiceProviderFactory::instance());
    for (size_t i(0); i < activeProviderTypes.size(); ++i)
      {
        SdoServiceProviderBase* svc
          = factory.createObject(activeProviderTypes[i]);
        
        SDOPackage::ServiceProfile prof;
        prof.id             = CORBA::string_dup(activeProviderTypes[i].c_str());
        prof.interface_type = CORBA::string_dup(activeProviderTypes[i].c_str());
        prof.service        = svc->_this();
        std::string propkey = ifrToKey(activeProviderTypes[i]);
        NVUtil::copyFromProperties(prof.properties,
                                   prop.getNode(propkey.c_str()));

        svc->init(rtobj, prof);
        m_providers.push_back(svc);
      }

    //------------------------------------------------------------
    // SDO service consumer
    // getting consumer types from RTC's properties

    ::std::string constypes = prop["sdo.service.consumer.enabled_services"];
    m_consumerTypes = ::coil::split(constypes, ",", true);
    RTC_DEBUG(("sdo.service.consumer.enabled_services: %s", constypes.c_str()));

    prop["sdo.service.consumer.available_services"]
      = coil::flatten(SdoServiceConsumerFactory::instance().getIdentifiers());
    RTC_DEBUG(("sdo.service.consumer.available_services: %s",
               prop["sdo.service.consumer.available_services"].c_str()));

    // If types include '[Aa][Ll][Ll]', all types enabled in this RTC
    for (size_t i(0); i < m_consumerTypes.size(); ++i)
      {
        std::string tmp(m_consumerTypes[i]);
        coil::toLower(tmp);
        if (tmp == "all")
          {
            m_allConsumerEnabled = true;
            RTC_DEBUG(("sdo.service.consumer.enabled_services: ALL"));
          }
      }
  }

  /*!
   * @if jp
   * @brief 仮想デストラクタ
   * @else
   * @brief Virtual destractor
   * @endif
   */
  SdoServiceAdmin::~SdoServiceAdmin()
  {
    for (size_t i(0); i < m_providers.size(); ++i)
      {
        m_providers[i]->finalize();
        delete m_providers[i];
      }
    m_providers.clear();

    for (size_t i(0); i < m_consumers.size(); ++i)
      {
        m_consumers[i]->finalize();
        delete m_consumers[i];
      }
    m_consumers.clear();
  }
  
  /*!
   * @if jp
   * @brief SDO Service Provider の ServiceProfileList を取得する
   * @else
   * @brief Get ServiceProfileList of SDO Service Provider
   * @endif
   */
  SDOPackage::ServiceProfileList* SdoServiceAdmin::getServiceProviderProfiles()
  {
    SDOPackage::ServiceProfileList_var prof
      = new SDOPackage::ServiceProfileList();
    Guard guard(m_provider_mutex);
    prof->length(m_providers.size());
    for (size_t i(0); i < m_providers.size(); ++i)
      {
        prof[i] = m_providers[i]->getProfile();
      }
    return prof._retn();
  }

  /*!
   * @if jp
   * @brief SDO Service Provider の ServiceProfile を取得する
   * @else
   * @brief Get ServiceProfile of an SDO Service Provider
   * @endif
   */
  SDOPackage::ServiceProfile*
  SdoServiceAdmin::getServiceProviderProfile(const char* id)
  {
    std::string idstr(id);
    Guard guard(m_provider_mutex);
    for (size_t i(0); i < m_providers.size(); ++i)
      {
        if (idstr == static_cast<const char*>(m_providers[i]->getProfile().id))
          {
            return new SDOPackage::ServiceProfile(m_providers[i]->getProfile());
          }
      }
    throw new SDOPackage::InvalidParameter();
    return new SDOPackage::ServiceProfile();
  }

  /*!
   * @if jp
   * @brief SDO Service Provider の Service を取得する
   * @else
   * @brief Get ServiceProfile of an SDO Service
   * @endif
   */   
  SDOPackage::SDOService_ptr SdoServiceAdmin::getServiceProvider(const char* id)
  {
    SDOPackage::ServiceProfile_var prof;
    prof = getServiceProviderProfile(id);
    SDOPackage::SDOService_var sdo 
      = SDOPackage::SDOService::_duplicate(prof->service);
    return sdo._retn();
  }

  /*!
   * @if jp
   * @brief SDO service provider をセットする
   * @else
   * @brief Set a SDO service provider
   * @endif
   */
  bool SdoServiceAdmin::
  addSdoServiceProvider(const SDOPackage::ServiceProfile& prof,
                        SdoServiceProviderBase* provider)
  {
    RTC_TRACE(("SdoServiceAdmin::addSdoServiceProvider(if=%s)",
               static_cast<const char*>(prof.interface_type)));
    Guard guard(m_provider_mutex);

    std::string id(static_cast<const char*>(prof.id));
    for (size_t i(0); i < m_providers.size(); ++i)
      {
        if (id == static_cast<const char*>(m_providers[i]->getProfile().id))
          {
            RTC_ERROR(("SDO service(id=%s, ifr=%s) already exists",
                       static_cast<const char*>(prof.id),
                       static_cast<const char*>(prof.interface_type)));
            return false;
          }
      }
    m_providers.push_back(provider);
    return true;
  }

  /*!
   * @if jp
   * @brief SDO service provider を削除する
   * @else
   * @brief Remove a SDO service provider
   * @endif
   */
  bool SdoServiceAdmin::removeSdoServiceProvider(const char* id)
  {
    RTC_TRACE(("removeSdoServiceProvider(%d)", id));
    Guard gurad(m_provider_mutex);

    std::string strid(id);
    std::vector<SdoServiceProviderBase*>::iterator it = m_providers.begin();
    std::vector<SdoServiceProviderBase*>::iterator it_end = m_providers.end();
    while (it != it_end)
      {
        if (strid == static_cast<const char*>((*it)->getProfile().id))
          {
            (*it)->finalize();
            SdoServiceProviderFactory& 
              factory(SdoServiceProviderFactory::instance());
            factory.deleteObject(*it);
            m_providers.erase(it);
            RTC_INFO(("SDO service provider has been deleted: %s", id));
            return true;
          }
        ++it;
      }
    RTC_WARN(("Specified SDO service provider not found: %s", id));
    return false;
  }

  /*!
   * @if jp
   * @brief Service Consumer を追加する
   * @else
   * @brief Add Service Consumer
   * @endif
   */
  bool SdoServiceAdmin::
  addSdoServiceConsumer(const SDOPackage::ServiceProfile& sProfile)
  {
    Guard guard(m_consumer_mutex);
    RTC_TRACE(("addSdoServiceConsumer(IFR = %s)",
               static_cast<const char*>(sProfile.interface_type)));
    
    // Not supported consumer type -> error return
    if (!isEnabledConsumerType(sProfile))  { return false; }
    if (!isExistingConsumerType(sProfile)) { return false; }
    RTC_DEBUG(("Valid SDO service required"));
    if (strncmp(sProfile.id, "", 1) == 0)   
      {
        RTC_WARN(("No id specified. It should be given by clients."));
        return false;
      }
    RTC_DEBUG(("Valid ID specified"));
    { // re-initialization
      std::string id(sProfile.id);
      for (size_t i(0); i < m_consumers.size(); ++i)
        {
          if (id == static_cast<const char*>(m_consumers[i]->getProfile().id))
            {
              RTC_INFO(("Existing consumer is reinitilized."));
              RTC_DEBUG(("Propeteis are: %s",
                         NVUtil::toString(sProfile.properties).c_str()));
              return m_consumers[i]->reinit(sProfile);
            }
        }
    }
    RTC_DEBUG(("SDO service properly initialized."));

    // new pofile
    SdoServiceConsumerFactory& 
      factory(SdoServiceConsumerFactory::instance());
    const char* ctype = static_cast<const char*>(sProfile.interface_type);
    if (ctype == NULL) { return false; }
    SdoServiceConsumerBase* consumer(factory.createObject(ctype));
    if (consumer == NULL) 
      {
        RTC_ERROR(("Hmm... consumer must be created."));
        return false; 
      }
    RTC_DEBUG(("An SDO service consumer created."));

    // initialize
    if (!consumer->init(m_rtobj, sProfile))
      {
        RTC_WARN(("SDO service initialization was failed."));
        RTC_DEBUG(("id:         %s", static_cast<const char*>(sProfile.id)));
        RTC_DEBUG(("IFR:        %s",
                   static_cast<const char*>(sProfile.interface_type)));
        RTC_DEBUG(("properties: %s",
                   NVUtil::toString(sProfile.properties).c_str()));
        factory.deleteObject(consumer);
        RTC_INFO(("SDO consumer was deleted by initialization failure"));
        return false;
      }
    RTC_DEBUG(("An SDO service consumer initialized."));
    RTC_DEBUG(("id:         %s", static_cast<const char*>(sProfile.id)));
    RTC_DEBUG(("IFR:        %s",
               static_cast<const char*>(sProfile.interface_type)));
    RTC_DEBUG(("properties: %s",
               NVUtil::toString(sProfile.properties).c_str()));

    // store consumer
    m_consumers.push_back(consumer);
    
    return true;
  }
    
  /*!
   * @if jp
   * @brief Service Consumer を削除する
   * @else
   * @brief Remove Service Consumer
   * @endif
   */
  bool SdoServiceAdmin::removeSdoServiceConsumer(const char* id)
  {
    Guard guard(m_consumer_mutex);
    if (id == NULL || id[0] == '\0')
      {
        RTC_ERROR(("removeSdoServiceConsumer(): id is invalid."));
        return false;
      }
    RTC_TRACE(("removeSdoServiceConsumer(id = %s)", id));

    std::string strid(id);
    std::vector<SdoServiceConsumerBase*>::iterator it = m_consumers.begin();
    std::vector<SdoServiceConsumerBase*>::iterator it_end = m_consumers.end();
    while (it != it_end)
      {
        if (strid == static_cast<const char*>((*it)->getProfile().id))
          {
            (*it)->finalize();
            SdoServiceConsumerFactory& 
              factory(SdoServiceConsumerFactory::instance());
            factory.deleteObject(*it);
            m_consumers.erase(it);
            RTC_INFO(("SDO service has been deleted: %s", id));
            return true;
          }
        ++it;
      }
    RTC_WARN(("Specified SDO consumer not found: %s", id));
    return false;
  }
    
  //------------------------------------------------------------
  // protected functios
  //------------------------------------------------------------

  /*!
   * @if jp
   * @brief 許可されたサービス型かどうか調べる
   * @else
   * @brief If it is enabled service type
   * @endif
   */
  bool SdoServiceAdmin::
  isEnabledConsumerType(const SDOPackage::ServiceProfile& sProfile)
  {
    if (m_allConsumerEnabled) { return true; }

    for (size_t i(0); i < m_consumerTypes.size(); ++i)
      {
        if (m_consumerTypes[i] == 
            static_cast<const char*>(sProfile.interface_type))
          {
            RTC_DEBUG(("%s is supported SDO service.",
                       static_cast<const char*>(sProfile.interface_type)));
            return true;
          }
      }
    RTC_WARN(("Consumer type is not supported: %s",
              static_cast<const char*>(sProfile.interface_type)));
    return false;
  }

  /*!
   * @if jp
   * @brief 存在するサービス型かどうか調べる
   * @else
   * @brief If it is existing service type
   * @endif
   */
  bool SdoServiceAdmin::
  isExistingConsumerType(const SDOPackage::ServiceProfile& sProfile)
  {
    SdoServiceConsumerFactory& factory(SdoServiceConsumerFactory::instance());
    coil::vstring consumerTypes(factory.getIdentifiers());
    
    for (size_t i(0); i < consumerTypes.size(); ++i)
      {
        if (consumerTypes[i] == 
            static_cast<const char*>(sProfile.interface_type))
          {
            RTC_DEBUG(("%s exists in the SDO service factory.",
                       static_cast<const char*>(sProfile.interface_type)));
            RTC_PARANOID(("Available SDO serices in the factory: %s",
                          coil::flatten(consumerTypes).c_str()));
            return true;
          }
      }
    RTC_WARN(("No available SDO service in the factory: %s",
              static_cast<const char*>(sProfile.interface_type)));
    return false;
  }

  const std::string SdoServiceAdmin::getUUID() const
  {
    coil::UUID_Generator uugen;
    uugen.init();
    std::auto_ptr<coil::UUID> uuid(uugen.generateUUID(2,0x01));
    
    return (const char*) uuid->to_string();
  }

  std::string SdoServiceAdmin::ifrToKey(std::string& ifr)
  {
    ::coil::vstring ifrvstr = ::coil::split(ifr, ":");
    ::coil::toLower(ifrvstr[1]);
    ::coil::replaceString(ifrvstr[1], ".", "_");
    ::coil::replaceString(ifrvstr[1], "/", ".");
    return ifrvstr[1];
  }


}; // end of namepsace RTC
