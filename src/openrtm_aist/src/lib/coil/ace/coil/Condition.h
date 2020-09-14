// -*- C++ -*-
/*!
 * @file  Condition_ace.h
 * @brief Condition variable class using ACE
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

#ifndef COIL_CONDITION_H
#define COIL_CONDITION_H

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
#include <coil/TimeValue.h>
#include <coil/Time.h>

namespace coil
{
  template <class Mutex>
  class Condition
  {
  public:
    Condition(Mutex& mutex)
      : m_mutex(mutex)
    {
      ACE_OS::cond_init(&m_cond, 0);
    }

    ~Condition()
    {
      ACE_OS::cond_destroy(&this->m_cond);
    }

    inline void signal()
    {
      m_mutex.trylock();
      ACE_OS::cond_signal(&m_cond);
    }

    inline void broadcast()
    {
      m_mutex.trylock();
      ACE_OS::cond_broadcast(&m_cond);
    }

    bool wait()
    {
      m_mutex.trylock();
      return 0 == ACE_OS::cond_wait(&m_cond, &m_mutex.mutex_);
    }

    bool wait(long second, long nano_second = 0)
    {
      m_mutex.trylock();
      coil::TimeValue now(coil::gettimeofday());
      coil::TimeValue abst(second, nano_second * 1000);
      abst += now;
      ACE_Time_Value abstime(abst.sec(), abst.usec());
      return 0 == ACE_OS::cond_timedwait(m_cond, &m_mutex.mutex_, &abstime);
    }

  private:
    Condition(const Mutex&);
    Condition& operator=(const Mutex &);
    ACE_cond_t m_cond;
    Mutex& m_mutex;
  };
};
#endif // COIL_CONDITION_H
