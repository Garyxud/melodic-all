// -*- C++ -*-
/*!
 * @file IceServantBase.h
 * @brief Doil Ice servant base class
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

#ifndef DOIL_ICE_ICESERVANTBASE_H
#define DOIL_ICE_ICESERVANTBASE_H

#include <rtm/config_rtc.h>
#include <doil/ServantBase.h>
#include <doil/ImplBase.h>
#include <doil/ice/Ice.h>

namespace doil
{
namespace Ice
{
  class IceServantBase
    : public virtual doil::ServantBase,
      public virtual ::Ice::Object
  {
  public:
    IceServantBase(ImplBase* impl)
      : m_refcount(1), m_id(impl->id()), m_name(impl->name())
    {
    }
    virtual ~IceServantBase(){}

    virtual const char* id() const
    {
      return m_id.c_str();
    }

    virtual const char* name() const
    {
      return m_name.c_str();
    }

    virtual void incRef()
    {
      ++m_refcount;
    }

    virtual void decRef()
    {
      --m_refcount;
      if (m_refcount == 0)
        delete this;
    }

  protected:
    int m_refcount;
    std::string m_id;
    std::string m_name;

  };
}; // namespoace doil
}; // namespace Ice
#endif // DOIL_ICE_ICESERVANTBASE_H

