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

namespace doil
{

  // Servant 生成・削除関数のtypedef
  typedef ServantBase* (*ServantNewFunc)();
  typedef void (*ServantDeleteFunc)(ServantBase*);

  // Servant 生成のためのテンプレート関数
  template <class Servant>
  ServantBase* New()
  {
    return new Servant();
  }

  // Servant 削除のためのテンプレート関数
  template <class Servant>
  void Delete(ServantBase* servant)
  {
    if (servant != NULL)
      {
        delete servant;
        servant == NULL;
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
    : public ServantFactoryBase
  {
  public:
    ServantFactory(const char* name, 
                   ServantNewFunc new_func,
                   ServantDeleteFunc delete_func);

    virtual ~ServantFactory();

    virtual const char* name();
    virtual ServantBase* create();
    virtual void destroy(ServantBase* servant);
  };


};
#endif // DOIL_SERVANTFACTORY_H
