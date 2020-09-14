// -*- C++ -*-
/*!
 * @file Time_posix.h
 * @brief Time functions
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

#ifndef COIL_TIME_H
#define COIL_TIME_H

#include <coil/config_coil.h>
#include <coil/TimeValue.h>
#if defined (WIN32)
#pragma warning( disable : 4244 ) 
#pragma warning( disable : 4312 ) 
#endif
#include <ace/OS_NS_sys_time.h>
#include <ace/OS_NS_unistd.h>
#include <ace/OS_NS_sys_select.h>
#if defined (WIN32)
#pragma warning( default : 4244 ) 
#pragma warning( default : 4312 ) 
#endif
#include <coil/TimeValue.h>

namespace coil
{

  inline unsigned int sleep(unsigned int seconds)
  {
    return ACE_OS::sleep(seconds);
  }

  inline int sleep(TimeValue& interval)
  {
    ACE_Time_Value tv(interval.sec(), interval.usec());
    return ACE_OS::select(0, 0, 0, 0, &tv);
  }

  inline int usleep(useconds_t usec)
  {
    ACE_Time_Value tv(0, usec);
    return ACE_OS::select(0, 0, 0, 0, &tv);
  }

  inline int gettimeofday(struct timeval *tv, struct timezone *tz)
  {
    ACE_Time_Value atv(ACE_OS::gettimeofday());
    if (tv != NULL)
      {
        tv->tv_sec  = static_cast<long int>(atv.sec());
        tv->tv_usec = static_cast<long int>(atv.usec());
      }
    return 0;
  }

  inline TimeValue gettimeofday()
  {
    ACE_Time_Value atv(ACE_OS::gettimeofday());
    return TimeValue(static_cast<long int>(atv.sec()),
                     static_cast<long int>(atv.usec()));
  }

  inline int settimeofday(const struct timeval *tv , const struct timezone *tz)
  {
    return 0; // no implementation
  }


};

#endif // COIL_TIME_H
