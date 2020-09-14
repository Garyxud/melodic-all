// -*- C++ -*-
/*!
 * @file  atomic.h
 * @brief atomic add func
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
#ifndef COIL_ATOMIC_H

#include <coil/Mutex.h>
#include <coil/Guard.h>
#define COIL_USES_ATOMIC_OP coil::Mutex __mutex;

#define atomic_add(x, y)                        \
  {                                             \
    coil::Guard<coil::Mutex> guard(__mutex);    \
    *x = *x + y;                                \
  }

#define atomic_incr(x)                          \
  {                                             \
    coil::Guard<coil::Mutex> guard(__mutex);    \
    ++(*x);                                     \
  }

#define atomic_decr(x)                          \
  {                                             \
    coil::Guard<coil::Mutex> guard(__mutex);    \
    --(*x);                                     \
  }

int exchange_add(int* x, int y, coil::Mutex* mutex)
{
  coil::Guard<coil::Mutex> guard(*mutex);
  int tmp(*x);
  *x = *x + y;
  return tmp;
}

#define atomic_exchange_add(x, y)               \
  exchange_add(x, y, &__mutex)
    
#endif // COIL_ATOMIC_H
