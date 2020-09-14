// -*- C++ -*-
/*!
 * @file  ExecutionContextAdapter.h
 * @brief ExecutionContextAdapter
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
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

#ifndef DOIL_CORBA_UTIL_H
#define DOIL_CORBA_UTIL_H

#include <doil/corba/CORBAManager.h>

namespace doil
{
namespace CORBA
{
  template <typename CORBAObject>
  typename CORBAObject::_ptr_type to_reference(ImplBase* impl)
  {
    ::CORBA::Object_ptr obj;
    obj = doil::CORBA::CORBAManager::instance().toReference(impl);
    return CORBAObject::_narrow(obj);
  }
};
};
#endif // DOIL_CORBA_UTIL_H
