// -*- C++ -*-
/*!
 * @file ServantBase.h
 * @brief doil implementation base class
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

#ifndef DOIL_SERVANTFACTORY_H
#define DOIL_SERVANTFACTORY_H

#include <doil/ServantBase.h>
#include <doil/ImplBase.h>

namespace doil
{
  // Servant 生成・削除関数のtypedef
  typedef ServantBase* (*ServantNewFunc)(ImplBase*);
  typedef void (*ServantDeleteFunc)(ServantBase*);

  // Servant 生成のためのテンプレート関数
  template <class Servant>
  ServantBase* New(ImplBase* impl)
  {
    return new Servant(impl);
  }

  // Servant 削除のためのテンプレート関数
  template <class Servant>
  void Delete(ServantBase* servant)
  {
    if (servant != NULL)
      {
        delete servant;
        servant = NULL;
      }
  }

  class ServantFactoryBase
  {
  public:
    virtual ~ServantFactoryBase(){}
    virtual const char* name() = 0;
    virtual ServantBase* create() = 0;
    virtual void destroy(ServantBase* servant) = 0;
  };


  class ServantFactory
  //    : public ServantFactoryBase
  {
  public:
    ServantFactory(const char* id, 
                   ServantNewFunc new_func,
                   ServantDeleteFunc delete_func)
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
    virtual ~ServantFactory(){};

    virtual const char* id()
    {
      return m_id.c_str();
    }
    virtual ServantBase* create(ImplBase* impl)
    {
      return m_new(impl);
    }
    virtual void destroy(ServantBase* servant)
    {
      m_delete(servant);
    }
  private:
    std::string m_id;
    ServantNewFunc m_new;
    ServantDeleteFunc m_delete;
  };


};
#endif // DOIL_SERVANTFACTORY_H
