// -*- C++ -*-
/*!
 * @file CorbaManager.cpp
 * @brief RTM CORBA manager class
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
#include <assert.h>
#include <iostream>
#include <coil/stringutil.h>
#include <doil/corba/CORBAManager.h>
#include <doil/ORBManager.h>

#define UNUSED_ARG(a) do {} while (&a == 0)

namespace doil
{
namespace CORBA
{
  // singleton pointer initializer
  CORBAManager* CORBAManager::manager = NULL;
  coil::Mutex CORBAManager::mutex;


  /*!
   * @if jp
   * @brief 初期化関数
   * @else
   * @brief initializer
   * @endif
   */
  CORBAManager* CORBAManager::init(coil::Properties prop)
    throw()
  {
    if (!manager)
      {
        coil::Guard<coil::Mutex> guard(mutex);
        if (!manager)
          {
            manager = new CORBAManager();
            manager->initOrb(prop);
          }
      }
    return manager;
  };

  /*!
   * @if jp
   * @brief インスタンス取得関数
   * @else
   * @brief getting instance
   * @endif
   */
  CORBAManager& CORBAManager::instance()
    throw()
  {
    coil::Properties prop;
    return *CORBAManager::init(prop);
  }

  /*!
   * @if jp
   * @brief ORB の名前を取得する
   * @else
   * @brief Getting ORB's name
   * @endif
   */
  const char* CORBAManager::name()
    throw()
  {
    return "corba";
  }
  void CORBAManager::run()
    throw()
  {
    // Do nothing
    return;
  }


  void CORBAManager::shutdown()
    throw()
  {
    while (m_orb->work_pending())
      {
        //        RTC_PARANOID(("Pending work still exists."));
        if (m_orb->work_pending())
          m_orb->perform_work();
      }
    
    if (!::CORBA::is_nil(m_poa))
      {
        try
          {
            if (!::CORBA::is_nil(m_poaManager))
              m_poaManager->deactivate(false, true);
            m_poa->destroy(false, true);
            m_poa = PortableServer::POA::_nil();
          }
        catch (::CORBA::SystemException& ex)
          {
          }
        catch (...)
          {
          }
      }
    
    if (!::CORBA::is_nil(m_orb))
      {
        try
          {
            m_orb->shutdown(true);
            m_orb = ::CORBA::ORB::_nil();
          }
        catch (::CORBA::SystemException& ex)
          {
            UNUSED_ARG(ex);
          }
        catch (...)
          {
          }
      }
    {
      coil::Guard<coil::Mutex> guard(mutex);
      delete manager;
      manager = NULL;
    }
  }


  /*!
   * @if jp
   * @brief Servant の Factory を登録する
   * @else
   * @brief Register servant's factory
   * @endif
   */
  ReturnCode_t CORBAManager::registerFactory(const char* id,
                                             doil::ServantNewFunc new_func,
                                             doil::ServantDeleteFunc delete_func)
    throw()
  {
    if (id == NULL) return INVALID_ARGS;
    if (new_func == NULL) return INVALID_ARGS;
    if (delete_func == NULL) return INVALID_ARGS;

    bool ret;
    ServantFactory* factory = new ServantFactory(id, new_func, delete_func);
    ret = m_factory.registerObject(factory);
    if (ret) return OK;
    else return ALREADY_EXISTS;
  }


  ReturnCode_t CORBAManager::registerProxyFactory(const char* id,
                                             doil::ProxyNewFunc new_func,
                                             doil::ProxyDeleteFunc delete_func)
    throw()
  {
    if (id == NULL) return INVALID_ARGS;
    if (new_func == NULL) return INVALID_ARGS;
    if (delete_func == NULL) return INVALID_ARGS;

    bool ret;
    ProxyFactory* factory = new ProxyFactory(id, new_func, delete_func);
    ret = m_factory_proxy.registerObject(factory);
    if (ret) return OK;
    else return ALREADY_EXISTS;
  }

  /*!
   * @if jp
   * @brief オブジェクトをactivateする
   * @else
   * @brief Activate object
   * @endif
   */
  ReturnCode_t CORBAManager::activateObject(doil::ImplBase* impl)
    throw()
  {
    if (impl == NULL) return INVALID_ARGS;

    const char* id = impl->id();
    std::cout << "CORBAManager::activateObject: id " << id << std::endl;
    std::cout << "CORBAManager::activateObject: name " << impl->name() << std::endl;
    ServantFactory* factory = m_factory.find(id);

#if 0
    if (factory != NULL)
    {
#else
    // Factory NOT_FOUND
    if (factory == NULL) return NOT_FOUND;
#endif
    // INVALID_ARGS
    try
      {
        doil::ServantBase* svt = factory->create(impl);
        CORBAServantBase* csvt = dynamic_cast<CORBAServantBase*>(svt);
        if (csvt == NULL) 
          {
            std::cout << "dynamic_cast<CORBAServantBase*> failed" << std::endl;
            delete svt;
            return INVALID_ARGS;
          }
        ReturnCode_t ret = activateObject(impl, csvt);
        if (ret != OK)
          delete svt;
        return ret;
      }
    catch (std::bad_alloc& e)
      {
        UNUSED_ARG(e);
        return INVALID_ARGS;
      }
    catch (...)
      {
        return UNKNOWN;
      }
#if 0
//
//for unit tests
//
    }
    else
    {
        ProxyFactory* factory_proxy = m_factory_proxy.find(id);
        if (factory_proxy == NULL) return NOT_FOUND;
        doil::ServantBase* svt = factory->create(impl);
    }
#endif
    return UNKNOWN;
  }

  /*!
   * @if jp
   * @brief オブジェクトをactivateする
   * @else
   * @brief Activate object
   * @endif
   */
  ReturnCode_t CORBAManager::activateObject(doil::ImplBase* impl,
                                            doil::ServantBase* servant)
    throw()
  {
    if (impl == NULL) return INVALID_ARGS;
    if (servant == NULL) return INVALID_ARGS;

    // check ID
    if (strcmp(impl->id(), servant->id()) != 0)
      return INVALID_ARGS;

    doil::CORBA::CORBAServantBase* svt;
    svt = dynamic_cast<doil::CORBA::CORBAServantBase*>(servant);
   
    if (svt == NULL) return INVALID_ARGS;

    // activate CORBA object
    ::PortableServer::ObjectId_var id = m_poa->activate_object(svt);
    ::CORBA::Object_ptr obj = m_poa->id_to_reference(id);

    Entry* entry = new Entry(impl, svt, obj);
    if (m_map.registerObject(entry)) return OK;
    return UNKNOWN;
  }
  
  /*!
   * @if jp
   * @brief オブジェクトをdeactivateする
   * @else
   * @brief Deactivate object
   * @endif
   */
  ReturnCode_t CORBAManager::deactivateObject(doil::ImplBase* impl)
    throw()
  {
    if (impl == NULL) return INVALID_ARGS;
    return deactivateObject(impl->name());
  }

  /*!
   * @if jp
   * @brief オブジェクトをdeactivateする
   * @else
   * @brief Deactivate object
   * @endif
   */
  ReturnCode_t CORBAManager::deactivateObject(const char* name)
    throw()
  {
    if (name == NULL) return INVALID_ARGS;

    Entry* entry = m_map.find(name);
    if (entry == NULL) return NOT_FOUND;
    if (::CORBA::is_nil(entry->objref_)) return NOT_FOUND;

    PortableServer::ObjectId_var oid;
    oid = m_poa->reference_to_id(entry->objref_);
    m_poa->deactivate_object(oid);

    ServantFactory* factory = m_factory.find(entry->servant_->id());
    factory->destroy(entry->servant_);
    m_map.unregisterObject(entry->impl_->name());
    delete entry;
    return OK;
  }

  /*!
   * @if jp
   * @brief Implオブジェクトを名前で取得する
   * @else
   * @brief Getting object by name
   * @endif
   */
  doil::ImplBase* CORBAManager::getImpl(const char* name)
    throw()
  {
    if (name == NULL) return NULL;

    Entry* entry = m_map.find(name);
    if (entry == NULL) return NULL;
    return entry->impl_;
  }

  /*!
   * @if jp
   * @brief ImplオブジェクトをServantで取得する
   * @else
   * @brief Getting impl object by servant
   * @endif
   */
  doil::ImplBase* CORBAManager::toImpl(doil::ServantBase* servant)
    throw()
  {
    if (servant == NULL) return NULL;

    std::cout << "toImpl(Servant)" << std::endl;
    CORBAServantBase* csvt = dynamic_cast<CORBAServantBase*>(servant);

    std::cout << "name: " << csvt->name() << std::endl;
    return getImpl(csvt->name());
  }

  /*!
   * @if jp
   * @brief Servantオブジェクトを取得する
   * @else
   * @brief Getting servant object by name
   * @endif
   */
  doil::ServantBase* CORBAManager::getServant(const char* name)
    throw()
  {
    if (name == NULL) return NULL;

    Entry* entry = m_map.find(name);
    if (entry == NULL) return NULL;
    return entry->servant_;

  }

  /*!
   * @if jp
   * @brief Servantオブジェクトを取得する
   * @else
   * @brief Getting servant object by impl object
   * @endif
   */
  doil::ServantBase* CORBAManager::toServant(doil::ImplBase* impl)
    throw()
  {
    if (impl == NULL) return NULL;
    return getServant(impl->name());
  }

  //------------------------------------------------------------
  // CORBAManager interfaces
  //------------------------------------------------------------
  /*!
   * @if jp
   * @brief Objectを関連付けられたImplに変換する
   * @else
   * @brief Convert CORBA Object_ptr to Impl related to it.
   * @endif
   */
  doil::ImplBase* CORBAManager::toImpl(::CORBA::Object_ptr obj)
    throw()
  {
    PortableServer::ServantBase* svt = m_poa->reference_to_servant(obj);
    CORBAServantBase* csvt = dynamic_cast<CORBAServantBase*>(svt);
    if (csvt == NULL) return NULL;
    return toImpl(csvt);
  }

  /*!
   * @if jp
   * @brief 名前からオブジェクト参照を取得する
   * @else
   * @brief Getting object reference from the given name
   * @endif
   */
  ::CORBA::Object_ptr CORBAManager::getReference(const char* name)
    throw()
  {
    Entry* entry = m_map.find(name);
    return entry->objref_;
    ;
  }

  /*!
   * @if jp
   * @brief Implオブジェクトからオブジェクト参照へ変換する
   * @else
   * @brief Converting Impl object to object reference
   * @endif
   */
  ::CORBA::Object_ptr CORBAManager::toReference(doil::ImplBase* impl)
    throw()
  {
    return getReference(impl->name());
  }

  /*!
   * @if jp
   * @brief Servantオブジェクトからオブジェクト参照へ変換する
   * @else
   * @brief Converting Servant object to object reference
   * @endif
   */
  ::CORBA::Object_ptr CORBAManager::toReference(doil::ServantBase* servant)
    throw()
  {
    return getReference(servant->name());
  }

  /*!
   * @if jp
   * @brief ORBのポインタを取得する
   *
   * @else
   * @brief Getting ORB pointer
   *
   * @endif
   */
  doil::ServantBase* CORBAManager::toServant(::CORBA::Object_ptr obj)
    throw()
  {
    PortableServer::ServantBase* svt = m_poa->reference_to_servant(obj);
    if (svt == NULL) return NULL;
    CORBAServantBase* csvt = dynamic_cast<CORBAServantBase*>(svt);
    if (csvt == NULL) return NULL;
    return csvt;
  }

  //------------------------------------------------------------
  // CORBA functions
  //------------------------------------------------------------
  /*!
   * @if jp
   * @brief ORBのポインタを取得する
   * @else
   * @brief Getting ORB pointer
   * @endif
   */
  ::CORBA::ORB_ptr CORBAManager::getORB()
    throw()
  {
    return m_orb;
  }

  /*!
   * @if jp
   * @brief デフォルトPOAのポインタを取得する
   * @else
   * @brief Getting default POA pointer
   * @endif
   */
  ::PortableServer::POA_ptr CORBAManager::getPOA()
    throw()
  {
    return m_poa;
  }

  /*!
   * @if jp
   * @brief POAManagerのポインタを取得する
   * @else
   * @brief Getting POAManager pointer
   * @endif
   */
  ::PortableServer::POAManager_ptr CORBAManager::getPOAManager()
    throw()
  {
    return m_poaManager;
  }


  //------------------------------------------------------------
  // protected:
  //------------------------------------------------------------
  void CORBAManager::initOrb(coil::Properties prop)
  {
    m_config = prop;
    std::vector<std::string> args(coil::split(createORBOptions(), " "));
    // TAO's ORB_init needs argv[0] as command name.
    args.insert(args.begin(), "manager");
    char** argv = coil::toArgv(args);
    int argc(args.size());

    try
      {
        
        m_orb = ::CORBA::ORB_init(argc, argv);
        assert(!::CORBA::is_nil(m_orb));
        
        ::CORBA::Object_var obj = m_orb->resolve_initial_references("RootPOA");
        m_poa = PortableServer::POA::_narrow(obj);
        assert(!::CORBA::is_nil(m_poa));
        m_poaManager = m_poa->the_POAManager();
	m_poaManager->activate();
      }
    catch (...)
      {
	return;
      }
  }


  std::string CORBAManager::createORBOptions()
  {
    std::string opt(m_config["args"]);
    std::string corba(m_config["id"]);
    std::string endpoint(m_config["endpoint"]);
    
    if (!endpoint.empty())
      {
	if (!opt.empty()) opt += " ";
	if (corba == "omniORB")   opt = "-ORBendPoint giop:tcp:" + endpoint;
	else if (corba == "TAO")  opt = "-ORBEndPoint iiop://" + endpoint;
	else if (corba == "MICO") opt = "-ORBIIOPAddr inet:" + endpoint;
      }
    return opt;
  }
};
};


extern "C"
{
  void DoilCORBAInit(coil::Properties& prop)
  {
    doil::ORBManager& mgr(doil::ORBManager::instance());
    mgr.addORB((doil::CORBA::CORBAManager::init(prop)));
  }
}
