// -*- C++ -*-
/*!
 * @file NamingManager.h
 * @brief naming Service helper class
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
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

#include <functional>
#include <algorithm>
#include <iostream>

#include <coil/Routing.h>
#include <coil/stringutil.h>

#include <rtm/NamingManager.h>
#include <rtm/Manager.h>
#include <rtm/CORBA_IORUtil.h>

namespace RTC
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  NamingOnCorba::NamingOnCorba(CORBA::ORB_ptr orb, const char* names)
    : m_cosnaming(orb, names), m_endpoint(""),
      m_replaceEndpoint(false)
  {
    rtclog.setName("NamingOnCorba");
    coil::Properties& prop(Manager::instance().getConfig());
    m_replaceEndpoint = 
      coil::toBool(prop["corba.nameservice.replace_endpoint"].c_str(),
                   "YES", "NO", true);


    coil::vstring host_port(coil::split(names, ":"));
    if (coil::dest_to_endpoint(host_port[0], m_endpoint))
      {
        RTC_INFO(("Endpoint for the CORBA naming service (%s) is %s.",
                  host_port[0].c_str(),
                  m_endpoint.c_str()));
      }
    else
      {
        RTC_WARN(("No endpoint for the CORBA naming service (%s) was found.",
                  host_port[0].c_str()));
      }
  }
  /*!
   * @if jp
   * @brief 指定した CORBA オブジェクトのNamingServiceへバインド
   * @else
   * @brief Bind the specified CORBA objects to NamingService
   * @endif
   */
  void NamingOnCorba::bindObject(const char* name,
				 const RTObject_impl* rtobj)
  {
    RTC_TRACE(("bindObject(name = %s, rtobj)", name));
#ifdef ORB_IS_OMNIORB
    if (!m_endpoint.empty() && m_replaceEndpoint)
      {
        CORBA::Object_var obj(RTObject::_duplicate(rtobj->getObjRef()));
        CORBA::String_var ior;
        ior = RTC::Manager::instance().getORB()->object_to_string(obj.in());
        std::string iorstr((const char*)ior);

        RTC_DEBUG(("Original IOR information:\n %s",
                   CORBA_IORUtil::formatIORinfo(iorstr.c_str()).c_str()));
        CORBA_IORUtil::replaceEndpoint(iorstr, m_endpoint);
        CORBA::Object_var newobj = RTC::Manager::instance().
          getORB()->string_to_object(iorstr.c_str());

        RTC_DEBUG(("Modified IOR information:\n %s",
                   CORBA_IORUtil::formatIORinfo(iorstr.c_str()).c_str()));
        m_cosnaming.rebindByString(name, newobj.in(), true);
      }
    else
      {
#endif // ORB_IS_OMNIORB
        m_cosnaming.rebindByString(name, rtobj->getObjRef(), true);
#ifdef ORB_IS_OMNIORB
      }
#endif // ORB_IS_OMNIORB
  }

  void NamingOnCorba::bindObject(const char* name,
				 const RTM::ManagerServant* mgr)
  {
    RTC_TRACE(("bindObject(name = %s, mgr)", name));
#ifdef ORB_IS_OMNIORB
    if (!m_endpoint.empty() && m_replaceEndpoint)
      {
        CORBA::Object_var obj(RTM::Manager::_duplicate(mgr->getObjRef()));
        CORBA::String_var ior;
        ior = RTC::Manager::instance().getORB()->object_to_string(obj.in());
        std::string iorstr((const char*)ior);

        RTC_DEBUG(("Original IOR information:\n %s",
                   CORBA_IORUtil::formatIORinfo(iorstr.c_str()).c_str()));
        CORBA_IORUtil::replaceEndpoint(iorstr, m_endpoint);
        CORBA::Object_var newobj = RTC::Manager::instance().
          getORB()->string_to_object(iorstr.c_str());

        RTC_DEBUG(("Modified IOR information]\n %s",
                   CORBA_IORUtil::formatIORinfo(iorstr.c_str()).c_str()));
        m_cosnaming.rebindByString(name, newobj.in(), true);
      }
    else
      {
#endif // ORB_IS_OMNIORB
        m_cosnaming.rebindByString(name, mgr->getObjRef(), true);
#ifdef ORB_IS_OMNIORB
      }
#endif // ORB_IS_OMNIORB
  }
  
  /*!
   * @if jp
   * @brief 指定した CORBA オブジェクトをNamingServiceからアンバインド
   * @else
   * @brief Unbind the specified CORBA object from NamingService
   * @endif
   */
  void NamingOnCorba::unbindObject(const char* name)
  {
    RTC_TRACE(("unbindObject(name  = %s)", name));
    m_cosnaming.unbind(name);
  }

  bool NamingOnCorba::isAlive()
  {
    RTC_TRACE(("isAlive()"));
    return m_cosnaming.isAlive();
  }

  
  //============================================================
  // NamingManager
  //============================================================
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  NamingManager::NamingManager(Manager* manager)
    :m_manager(manager), rtclog("NamingManager")
  {
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  NamingManager::~NamingManager()
  {
  }
  
  /*!
   * @if jp
   * @brief NameServer の登録
   * @else
   * @brief Register the NameServer
   * @endif
   */
  void NamingManager::registerNameServer(const char* method,
					 const char* name_server)
  {
    RTC_TRACE(("NamingManager::registerNameServer(%s, %s)",
	       method, name_server));
    NamingBase* name;
    name = createNamingObj(method, name_server);

    Guard guard(m_namesMutex);
    m_names.push_back(new Names(method, name_server, name));
  }
  
  /*!
   * @if jp
   * @brief 指定したオブジェクトのNamingServiceへバインド
   * @else
   * @brief Bind the specified objects to NamingService
   * @endif
   */
  void NamingManager::bindObject(const char* name, 
				 const RTObject_impl* rtobj)
  {
    RTC_TRACE(("NamingManager::bindObject(%s)", name));
    
    Guard guard(m_namesMutex);
    for (int i(0), len(m_names.size()); i < len; ++i)
      {
	if (m_names[i]->ns != 0)
          {
            try
              {
                m_names[i]->ns->bindObject(name, rtobj);
              }
            catch (...)
              {
                delete m_names[i]->ns;
                m_names[i]->ns = 0;
              }
          }
      }
    registerCompName(name, rtobj);
  }
  void NamingManager::bindObject(const char* name, 
				 const RTM::ManagerServant* mgr)
  {
    RTC_TRACE(("NamingManager::bindObject(%s)", name));
    
    Guard guard(m_namesMutex);
    for (int i(0), len(m_names.size()); i < len; ++i)
      {
	if (m_names[i]->ns != 0)
          {
            try
              {
                m_names[i]->ns->bindObject(name, mgr);
              }
            catch (...)
              {
                delete m_names[i]->ns;
                m_names[i]->ns = 0;
              }
          }
      }
    registerMgrName(name, mgr);
  }
  
  /*!
   * @if jp
   * @brief NamingServer の情報の更新
   * @else
   * @brief Update information of NamingServer
   * @endif
   */
  void NamingManager::update()
  {
    RTC_TRACE(("NamingManager::update()"));

    Guard guard(m_namesMutex);
    bool rebind(coil::toBool(m_manager->getConfig()["naming.update.rebind"],
                             "YES", "NO", false));
    for (int i(0), len(m_names.size()); i < len; ++i)
      {
	if (m_names[i]->ns == 0) // if ns==NULL
	  {
            RTC_DEBUG(("Retrying connection to %s/%s",
                       m_names[i]->method.c_str(),
                       m_names[i]->nsname.c_str()));
            retryConnection(m_names[i]);
	  }
        else
          {	
            try
              {
                if (rebind) { bindCompsTo(m_names[i]->ns); }
                if (!m_names[i]->ns->isAlive())
                  {
                    RTC_INFO(("Name server: %s (%s) disappeared.",
                              m_names[i]->nsname.c_str(),
                              m_names[i]->method.c_str()));  
                    delete m_names[i]->ns;
                    m_names[i]->ns = 0;
                  }
              }
            catch (...)
              {
                RTC_INFO(("Name server: %s (%s) disappeared.",
                          m_names[i]->nsname.c_str(),
                          m_names[i]->method.c_str()));
                delete m_names[i]->ns;
                m_names[i]->ns = 0;
              } 
          } 
      }
  }
  
  /*!
   * @if jp
   * @brief 指定したオブジェクトをNamingServiceからアンバインド
   * @else
   * @brief Unbind the specified object from NamingService
   * @endif
   */
  void NamingManager::unbindObject(const char* name)
  {
    RTC_TRACE(("NamingManager::unbindObject(%s)", name));
    
    Guard guard(m_namesMutex);
    for (int i(0), len(m_names.size()); i < len; ++i)
      {
	if (m_names[i]->ns != NULL)
        {
	  m_names[i]->ns->unbindObject(name);
        }
      }
    unregisterCompName(name);
    unregisterMgrName(name);
  }
  
  /*!
   * @if jp
   * @brief 全てのオブジェクトをNamingServiceからアンバインド
   * @else
   * @brief Unbind all objects from NamingService
   * @endif
   */
  void NamingManager::unbindAll()
  {
    RTC_TRACE(("NamingManager::unbindAll(): %d names.", m_compNames.size()));
    {
      Guard guard(m_compNamesMutex);
      coil::vstring names;
      // unbindObject modifiy m_compNames
      for (int i(0), len(m_compNames.size()); i < len; ++i)
        {
          names.push_back(m_compNames[i]->name);
        }
      for (size_t i(0); i < names.size(); ++i)
        {
          unbindObject(names[i].c_str());
        }

    }
    {
      Guard guard(m_mgrNamesMutex);
      coil::vstring names;
      // unbindObject modifiy m_mgrNames
      for (int i(0), len(m_mgrNames.size()); i < len; ++i)
        {
          names.push_back(m_mgrNames[i]->name);
        }
      for (size_t i(0); i < names.size(); ++i)
        {
          unbindObject(names[i].c_str());
        }
    }
  }
  
  /*!
   * @if jp
   * @brief バインドされている全てのオブジェクトを取得
   * @else
   * @brief Get all bound objects
   * @endif
   */
  std::vector<RTObject_impl*> NamingManager::getObjects()
  {
    std::vector<RTObject_impl*> comps;
    Guard guard(m_compNamesMutex);
    
    for (int i(0), len(m_compNames.size()); i < len; ++i)
      {
	comps.push_back(const_cast<RTObject_impl*>(m_compNames[i]->rtobj));
      }
    return comps;
  }
  
  //============================================================
  // Protected
  //============================================================
  /*!
   * @if jp
   * @brief NameServer 管理用オブジェクトの生成
   * @else
   * @brief Create objects for NameServer management
   * @endif
   */
  NamingBase* NamingManager::createNamingObj(const char* method,
					     const char* name_server)
  {
    RTC_TRACE(("createNamingObj(method = %s, nameserver = %s",
               method, name_server));
    std::string m(method);
    if (m == "corba")
      {
	try
	  {
	    NamingBase* name;
            CORBA::ORB_var orb;
            orb = CORBA::ORB::_duplicate(m_manager->getORB());
	    name = new NamingOnCorba(orb.in(), name_server);
	    if (name == NULL) return NULL;
	    RTC_INFO(("NameServer connection succeeded: %s/%s",		\
		      method, name_server));
	    return name;
	  }
	catch (...)
	  {
	    RTC_INFO(("NameServer connection failed: %s/%s",		\
		      method, name_server));
	    return NULL;
	  }
      }
    return NULL;
  }
  
  /*!
   * @if jp
   * @brief 設定済みコンポーネントを NameServer に登録
   * @else
   * @brief Register the configured component to NameServer
   * @endif
   */
  void NamingManager::bindCompsTo(NamingBase* ns)
  {
    for (int i(0), len(m_compNames.size()); i < len; ++i)
      {
	ns->bindObject(m_compNames[i]->name.c_str(), m_compNames[i]->rtobj);
      }
  }
  
  /*!
   * @if jp
   * @brief NameServer に登録するコンポーネントの設定
   * @else
   * @brief Configure the components that will be registered to NameServer
   * @endif
   */
  void NamingManager::registerCompName(const char* name,
				       const RTObject_impl* rtobj)
  {
    for (int i(0), len(m_compNames.size()); i < len; ++i)
      {
	if (m_compNames[i]->name == name)
	  {
	    m_compNames[i]->rtobj = rtobj;
	    return;
	  }
      }
    m_compNames.push_back(new Comps(name, rtobj));
    return;
  }
  void NamingManager::registerMgrName(const char* name,
                                      const RTM::ManagerServant* mgr)
  {
    for (int i(0), len(m_mgrNames.size()); i < len; ++i)
      {
	if (m_mgrNames[i]->name == name)
	  {
	    m_mgrNames[i]->mgr = mgr;
	    return;
	  }
      }
    m_mgrNames.push_back(new Mgr(name, mgr));
    return;
  }
  
  /*!
   * @if jp
   * @brief NameServer に登録するコンポーネントの設定解除
   * @else
   * @brief Unregister the components that will be registered to NameServer
   * @endif
   */
  void NamingManager::unregisterCompName(const char* name)
  {
    std::vector<Comps*>::iterator it(m_compNames.begin());
    for (int i(0), len(m_compNames.size()); i < len; ++i, ++it)
      {
	if (m_compNames[i]->name == name)
	  {
	    delete m_compNames[i];
	    m_compNames.erase(it);
	    return;
	  }
      }
    return;
  }
  void NamingManager::unregisterMgrName(const char* name)
  {
    std::vector<Mgr*>::iterator it(m_mgrNames.begin());
    for (int i(0), len(m_mgrNames.size()); i < len; ++i, ++it)
      {
	if (m_mgrNames[i]->name == name)
	  {
	    delete m_mgrNames[i];
	    m_mgrNames.erase(it);
	    return;
	  }
      }
    return;
  }

  void NamingManager::retryConnection(Names* ns)
  {
    // recreate NamingObj
    NamingBase* nsobj(0);
    try
      {
        nsobj = createNamingObj(ns->method.c_str(),
                                ns->nsname.c_str());
        if (nsobj != 0) // if succeed
          {
            RTC_INFO(("Connected to a name server: %s/%s",
                      ns->method.c_str(), ns->nsname.c_str()));
            ns->ns = nsobj;
            bindCompsTo(nsobj); // rebind all comps to new NS
            return;
          }
        else
          {
            RTC_DEBUG(("Name service: %s/%s still not available.",
                       ns->method.c_str(),
                       ns->nsname.c_str()));
          }
      }
    catch (...)
      {
        RTC_DEBUG(("Name server: %s/%s disappeared again.",
                   ns->method.c_str(),
                   ns->nsname.c_str()));
        if (nsobj != 0)
          {
            delete ns->ns;
            ns->ns = 0;
          } 
      }
  }
}; // namespace RTC
