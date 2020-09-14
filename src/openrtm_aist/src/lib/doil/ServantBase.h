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

#ifndef DOIL_SERVANTBASE_H
#define DOIL_SERVANTBASE_H

namespace doil
{
  class ServantBase
  {
  public:
    virtual ~ServantBase() {};
    virtual const char* id() const = 0;
    virtual const char* name() const = 0;
    virtual void incRef() = 0;
    virtual void decRef() = 0;
  };
};
#endif // DOIL_SERVANTBASE_H
