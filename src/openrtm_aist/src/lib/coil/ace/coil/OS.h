// -*- C++ -*-
/*!
 * @file Task.cpp
 * @brief Task class
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

#ifndef COIL_OS_H
#define COIL_OS_H

#include <ace/OS.h>
#include <ace/Get_Opt.h>

extern "C"
{
  extern char *optarg;
};

namespace coil
{
  typedef ACE_utsname utsname;
  inline int uname(utsname* name)
  {
    return ACE_OS::uname(name);
  }

  typedef ::pid_t pid_t;
  inline pid_t getpid()
  {
    return ACE_OS::getpid();
  }
  inline pid_t getppid()
  {
    return ACE_OS::getppid();
  }

  inline char* getenv(const char *name)
  {
    return ACE_OS::getenv(name);
  }

  typedef ACE_Get_Opt GetOpt;
};

#endif // COIL_OS_H
