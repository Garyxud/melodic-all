// -*- C++ -*-
/*!
 * @file  Mutex_ace.h
 * @brief Mutex class using ACE
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

#ifndef COIL_MUTEX_H
#define COIL_MUTEX_H

#include <coil/config_coil.h>
#if defined (WIN32)
#pragma warning( disable : 4244 ) 
#pragma warning( disable : 4312 ) 
#endif
#include <ace/OS_NS_Thread.h>
#if defined (WIN32)
#pragma warning( default : 4244 ) 
#pragma warning( default : 4312 ) 
#endif

namespace coil
{
  class Mutex
  {
  public:
    Mutex(const char* naem = 0)
    {
      ACE_OS::thread_mutex_init(&mutex_, 0);
    }

    ~Mutex()
    {
      ACE_OS::thread_mutex_destroy(&mutex_);
    }

    inline void lock()
    {
      ACE_OS::thread_mutex_lock(&mutex_);
    }

    inline bool trylock()
    {
      return ACE_OS::thread_mutex_trylock(&mutex_);
    }

    inline void unlock()
    {
      ACE_OS::thread_mutex_unlock(&mutex_);
    }
    ACE_thread_mutex_t mutex_;

  private:
    Mutex(const Mutex&);
    Mutex& operator=(const Mutex &);
  };
};
#endif // COIL_MUTEX_H
