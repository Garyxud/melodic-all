// -*- C++ -*-
/*!
 * @file ImplBase.h
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

#ifndef DOIL_IMPLEBASE_H
#define DOIL_IMPLEBASE_H

namespace doil
{
  class ImplBase
  {
  public:
    virtual ~ImplBase(){};
    virtual const char* id() = 0;
    virtual const char* name() = 0;
    virtual void incRef() = 0;
    virtual void decRef() = 0;
  };

  typedef ImplBase LocalBase;
};
#endif // DOIL_IMPLEBASE_H
