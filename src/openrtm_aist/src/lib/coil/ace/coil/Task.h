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

#ifndef COIL_TASK_H
#define COIL_TASK_H

#include <coil/config_coil.h>
#if defined (WIN32)
#pragma warning( disable : 4244 ) 
#pragma warning( disable : 4312 ) 
#endif
#include <ace/Task.h>
#if defined (WIN32)
#pragma warning( default : 4244 ) 
#pragma warning( default : 4312 ) 
#endif
namespace coil
{
  typedef ACE_Task<ACE_MT_SYNCH> Task;
};

#endif // COIL_TASK_H
