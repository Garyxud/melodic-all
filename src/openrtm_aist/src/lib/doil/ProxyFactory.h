// -*- C++ -*-
/*!
 * @file ProxyFactory.h
 * @brief doil implementation base class for debag
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

#ifndef DOIL_PROXYFACTORY_H
#define DOIL_PROXYFACTORY_H

#include <doil/ProxyBase.h>
#include <doil/corba/CORBA.h>

namespace doil
{
  // Servant 生成・削除関数のtypedef
//  typedef ImplBase* (*ProxyNewFunc)(::CORBA::Object_ptr obj);
//  typedef void (*ProxyDeleteFunc)(ImplBase*);
  typedef ProxyBase* (*ProxyNewFunc)(::CORBA::Object_ptr obj);
  typedef void (*ProxyDeleteFunc)(ProxyBase*);

  // Servant 生成のためのテンプレート関数
  template <class Proxy>
  ProxyBase* New(::CORBA::Object_ptr obj)
  {
    return new Proxy(obj);
  }

  // Servant 削除のためのテンプレート関数
  template <class Proxy>
//  void Delete(ImplBase* impl)
  void Delete(ProxyBase* impl)
  {
    if (impl != NULL)
      {
        delete impl;
        impl = NULL;
      }
  }

  class ProxyFactoryBase
  {
  public:
    virtual ~ProxyFactoryBase(){}
    virtual const char* name() = 0;
    virtual ProxyBase* create() = 0;
//    virtual void destroy(ImplBase* impl) = 0;
    virtual void destroy(ProxyBase* impl) = 0;
  };


  class ProxyFactory
  //    : public ServantFactoryBase
  {
  public:
    ProxyFactory(const char* id, 
                   ProxyNewFunc new_func,
                   ProxyDeleteFunc delete_func)
      : m_id(id), m_new(new_func), m_delete(delete_func)
    {
    }

    /*
    ServantFactory(const ServantFactory& sf)
    {
      m_name   = sf.m_name;
      m_new    = sf.m_new;
      m_delete = sf.m_delete;
    }

    ServantFactory& operator=(ServantFactory& sf)
    {
      ServantFactory tmp(sf);
      std::swap(*this, tmp);
      return *this;
    }
    */
    virtual ~ProxyFactory(){};

    virtual const char* id()
    {
      return m_id.c_str();
    }
    virtual ProxyBase* create(::CORBA::Object_ptr obj)
    {
      return m_new(obj);
    }
//    virtual void destroy(ImplBase* impl)
    virtual void destroy(ProxyBase* impl)
    {
      m_delete(impl);
    }
  private:
    std::string m_id;
    ProxyNewFunc m_new;
    ProxyDeleteFunc m_delete;
  };


};
#endif // DOIL_SERVANTFACTORY_H

