// -*- C++ -*-
/*!
 * @file CorbaManager.cpp
 * @brief RTM Ice manager class
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
#include <doil/ice/IceManager.h>
#include <doil/ORBManager.h>

#define UNUSED_ARG(a) do {} while (&a == 0)

namespace doil
{
namespace Ice
{
  // singleton pointer initializer
  IceManager* IceManager::manager = NULL;
  coil::Mutex IceManager::mutex;


  /*!
   * @if jp
   * @brief 初期化関数
   * @else
   * @brief initializer
   * @endif
   */
  IceManager* IceManager::init(coil::Properties prop)
    throw()
  {
    if (!manager)
      {
        coil::Guard<coil::Mutex> guard(mutex);
        if (!manager)
          {
            manager = new IceManager();
            manager->initIce(prop);
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
  IceManager& IceManager::instance()
    throw()
  {
    coil::Properties prop;
    return *IceManager::init(prop);
  }

  /*!
   * @if jp
   * @brief ORB の名前を取得する
   * @else
   * @brief Getting ORB's name
   * @endif
   */
  const char* IceManager::name()
    throw()
  {
    return "ice";
  }
  void IceManager::run()
    throw()
  {
    // Do nothing
    return;
  }


  void IceManager::shutdown()
    throw()
  {
    if (m_ice)
      {
        try
          {
            m_ice->destroy();
          }
        catch (const ::Ice::Exception& e)
          {
            std::cerr << e << std::endl;
          }
      }
  }


  /*!
   * @if jp
   * @brief Servant の Factory を登録する
   * @else
   * @brief Register servant's factory
   * @endif
   */
  ReturnCode_t IceManager::registerFactory(const char* id,
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

  /*!
   * @if jp
   * @brief オブジェクトをactivateする
   * @else
   * @brief Activate object
   * @endif
   */
  ReturnCode_t IceManager::activateObject(doil::ImplBase* impl)
    throw()
  {
    if (impl == NULL) return INVALID_ARGS;

    const char* id = impl->id();
    std::cout << "IceManager::activateObject: id " << id << std::endl;
    std::cout << "IceManager::activateObject: name " << impl->name() << std::endl;
    ServantFactory* factory = m_factory.find(id);

    // Factory NOT_FOUND
    if (factory == NULL) return NOT_FOUND;

    try
      {
        // INVALID_ARGS
        doil::ServantBase* svt = factory->create(impl);
        IceServantBase* csvt = dynamic_cast<IceServantBase*>(svt);
        if (csvt == NULL) 
          {
            std::cout << "dynamic_cast<IceServantBase*> failed" << std::endl;
            delete svt;
            return INVALID_ARGS;
          }
        return activateObject(impl, csvt);
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
    return UNKNOWN;
  }

  /*!
   * @if jp
   * @brief オブジェクトをactivateする
   * @else
   * @brief Activate object
   * @endif
   */
  ReturnCode_t IceManager::activateObject(doil::ImplBase* impl,
                                          doil::ServantBase* servant)
    throw()
  {
    if (impl == NULL) return INVALID_ARGS;
    if (servant == NULL) return INVALID_ARGS;

    // check ID
    if (strcmp(impl->id(), servant->id()) != 0)
      return INVALID_ARGS;

    doil::Ice::IceServantBase* svt;
    svt = dynamic_cast<doil::Ice::IceServantBase*>(servant);
   
    if (svt == NULL) return INVALID_ARGS;

    // activate Ice object
    ::Ice::ObjectPrx obj;
    obj = m_adapter->add(svt,
                   m_ice->stringToIdentity(impl->name()));

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
  ReturnCode_t IceManager::deactivateObject(doil::ImplBase* impl)
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
  ReturnCode_t IceManager::deactivateObject(const char* name)
    throw()
  {
    if (name == NULL) return INVALID_ARGS;

    Entry* entry = m_map.find(name);
    if (entry == NULL) return NOT_FOUND;
    // ### objref should be checked if nil reference
    //    if (::Ice::is_nil(entry->objref_)) return NOT_FOUND;

    ::Ice::ObjectPtr obj;
    obj = m_adapter->remove(m_ice->stringToIdentity(name));
    // ### obj should be deleted.

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
  doil::ImplBase* IceManager::getImpl(const char* name)
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
  doil::ImplBase* IceManager::toImpl(doil::ServantBase* servant)
    throw()
  {
    if (servant == NULL) return NULL;

    std::cout << "toImpl(Servant)" << std::endl;
    IceServantBase* csvt = dynamic_cast<IceServantBase*>(servant);

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
  doil::ServantBase* IceManager::getServant(const char* name)
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
  doil::ServantBase* IceManager::toServant(doil::ImplBase* impl)
    throw()
  {
    if (impl == NULL) return NULL;
    return getServant(impl->name());
  }

  //------------------------------------------------------------
  // IceManager interfaces
  //------------------------------------------------------------
  /*!
   * @if jp
   * @brief Objectを関連付けられたImplに変換する
   * @else
   * @brief Convert Ice ObjectPrx to Impl related to it.
   * @endif
   */
  doil::ImplBase* IceManager::toImpl(::Ice::ObjectPrx obj)
    throw()
  {
    find_by_obj p(obj);
    p = m_map.for_each(p);
    if (p.result_ == NULL) return NULL;
    return p.result_->impl_;
  }

  /*!
   * @if jp
   * @brief 名前からオブジェクト参照を取得する
   * @else
   * @brief Getting object reference from the given name
   * @endif
   */
  ::Ice::ObjectPrx IceManager::getReference(const char* name)
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
  ::Ice::ObjectPrx IceManager::toReference(doil::ImplBase* impl)
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
  ::Ice::ObjectPrx IceManager::toReference(doil::ServantBase* servant)
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
  doil::ServantBase* IceManager::toServant(::Ice::ObjectPrx obj)
    throw()
  {
    return toServant(toImpl(obj));
  }

  //------------------------------------------------------------
  // Ice functions
  //------------------------------------------------------------
  /*!
   * @if jp
   * @brief ORBのポインタを取得する
   * @else
   * @brief Getting ORB pointer
   * @endif
   */
  ::Ice::CommunicatorPtr IceManager::getORB()
    throw()
  {
    return m_ice;
  }

  /*!
   * @if jp
   * @brief デフォルトPOAのポインタを取得する
   * @else
   * @brief Getting default POA pointer
   * @endif
   */
  ::Ice::ObjectAdapterPtr IceManager::getAdapter()
    throw()
  {
    return m_adapter;
  }


  //------------------------------------------------------------
  // protected:
  //------------------------------------------------------------
  void IceManager::initIce(coil::Properties prop)
  {
    m_config = prop;
    std::vector<std::string> args(coil::split(createIceOptions(), " "));
    // TAO's ORB_init needs argv[0] as command name.
    args.insert(args.begin(), "manager");
    char** argv = coil::toArgv(args);
    int argc(args.size());

    try
      {
        m_ice = ::Ice::initialize(argc, argv);
        //        assert(!::Ice::is_nil(m_orb));
        m_adapter = m_ice->createObjectAdapterWithEndpoints(
                                                            "OpenRTM", 
                                                            "default -p 10000");
	m_adapter->activate();
      }
    catch (...)
      {
	return;
      }
  }


  std::string IceManager::createIceOptions()
  {
    std::string opt(m_config["args"]);
    return opt;
  }
};
};

extern "C"
{
  void DoilIceInit(coil::Properties& prop)
  {
    doil::ORBManager& mgr(doil::ORBManager::instance());
    mgr.addORB((doil::Ice::IceManager::init(prop)));
  }
}
